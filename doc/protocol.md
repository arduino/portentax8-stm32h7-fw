X8/H7 Protocol
==============

## Glossary

- AP (Application Processor, in this case the i.MX8 M Mini processor from NXP)
- H7 (MCU, Micro Controller Unit) is the STM32H747xI/G device connected via SPI to the AP as real-time co-processor

## Hardware / Signals

Please see full devicetree [here](https://github.com/arduino/meta-partner-arduino/blob/main/recipes-bsp/device-tree/lmp-device-tree/portenta-x8/ov_som_x8h7.dts)

```
pinctrl_irq_x8h7: x8h7irqgrp {
    fsl,pins = <
        /* H7 interrupt pin active low, pullup */
        MX8MM_IOMUXC_GPIO1_IO09_GPIO1_IO9		0x151 /* PC1_STM32 */
    >;
};

pinctrl_gpio_x8h7: gpiox8h7grp {
    fsl,pins = <
        MX8MM_IOMUXC_GPIO1_IO10_GPIO1_IO10		0x151 /* NRST_STM32 Pull Up */
        MX8MM_IOMUXC_GPIO1_IO11_GPIO1_IO11		0x110 /* BOOT0_STM32 Pull Down */
        MX8MM_IOMUXC_GPIO1_IO08_GPIO1_IO8		0x110 /* SWDIO_STM32 Pull Down */
        MX8MM_IOMUXC_GPIO1_IO15_GPIO1_IO15		0x110 /* SWCLK_STM32 Pull Down */
        MX8MM_IOMUXC_GPIO1_IO07_GPIO1_IO7		0x110 /* PA0_STM32 Pull Down M4 led red */
    >;
};

pinctrl_ecspi3: ecspi3grp {
    fsl,pins = <
        MX8MM_IOMUXC_UART1_RXD_ECSPI3_SCLK		0x82
        MX8MM_IOMUXC_UART1_TXD_ECSPI3_MOSI		0x82
        MX8MM_IOMUXC_UART2_RXD_ECSPI3_MISO		0x82
    >;
};

pinctrl_ecspi3_cs: ecspi3cs {
    fsl,pins = <
        MX8MM_IOMUXC_UART2_TXD_ECSPI3_SS0		0x40000
    >;
};
```

## SPI

The data exchanged between the AP and H7 has the layout of one superframe which can consist of zero, one or multiple subframes.
The purpose behind this design decision it to enqueue multiple IO requests/responses withing a single SPI transfer in order to reduce the overall amount of SPI transfers happening.

Moreover, this superframe is sent in two dinstinct SPI transfers: the header and the effective payload. The reason behind this decision is to include the length of the superframe
in a short header which is always sent first, so that both devices can configure their respective dma channels to match the number of bytes expected. The only other available solution would
have been to use a fixed packet length: this was discarded by us since it would have required to either pad with zeros / empty or split the pending queue in multiple transfers.

### Protocol Details

The SPI transfer is always initiated by the AP. It can happens when the queue in the main x8h7_drv.c kernel module is not zero - this typically means one or more peripheral subdrivers has been
ioctl'ed - or an irq event from H7 has been received. This condition is defined as a superframe transmission event.

When a superframe transmission event is invoked, the AP sends the header packet over SPI first.

### Header Description

| Name | Byte | Size / Bytes | Description |
|:-:|:-:|:-:|-|
| `size` | 0-1 | 2 | Total superframe length in bytes (max. 2048 bytes) |
| `checksum` | 2-3 | 2 | Checksum calculated by xor-ing super frame length with `0x5555`. |

When the above transaction has ended, the AP parses the SPI rx buffer and retrieve the length of the superframe that the H7 needs to send to the AP eventually (can be zero)
and then a max of the twos is done

len = max(AP superframe length, H7 superframe length)

at this point the superframe is sent / received on both devices, which should have configured their dmas BEFORE the 2nd CS falling edge.

The above mechanism is implemented notably in the [source code of our kernel driver](https://github.com/arduino/meta-partner-arduino/blob/82ca2ead7f129e55df3314bb9ff4391784bc4c29/recipes-kernel/kernel-modules/x8h7/x8h7_drv.c#L310)

### Superframe Description

| Name | Byte | Size / Bytes | Description |
|:-:|:-:|:-:|-|
| `data` | 4-(4+n) | n | Subframe #1 |
| `data` | (4+n+1)-(4+n+1+m)| m | Subframe #2 |
| `data` | ... | ... | ... |

### Subframe Description

| Name | Byte | Size / Bytes | Description |
|:-:|:-:|:-:|-|
| `peripheral` | 0 | 1 | Peripheral identifier code, i.e. `0x01` = ADC |
| `opcode` | 1 | 1 | Opcode described the desired operation to be executed on peripheral |
| `size` | 2-3 | 2 | Number of bytes contained within subframe data (`n`) |
| `data` | 4-n | n | Subframe data |

## Peripherals

### ADC (`0x01`)

**Note**: Configuration of sample rate not implemented, ADC uses polling mode whenever trying to obtain latest ADC measurement.
| `opcode` | `toStr(opcode)` | `size` / Bytes | `data` | `toStr(data)` |
|:-:|:-:|:-:|-|:-:|
| `0x01`| GET_RAW_CHANNEL_1 | 2 | `uint16_t adc_ch_1_raw;` | Raw ADC value obtained from a one-time polled measurement of ADC channel #1 |
| `0x02`| GET_RAW_CHANNEL_2 | 2 | `uint16_t adc_ch_2_raw;` | Raw ADC value ... channel #2 |
| ... | ... | ... | ... |
| `0x08`| GET_RAW_CHANNEL_8 | 2 | `uint16_t adc_ch_8_raw;` | Raw ADC value ... channel #8 |
| `0x10`| CONFIG_SAMPLE_RATE | 2 | `uint16_t adc_sample_rate_Hz;`  | Desired sample rate in Hz |

### PWM (`0x02`)

| `opcode` | `toStr(opcode)` | `size` / Bytes | `data` | `toStr(data)` |
|:-:|:-:|:-:|-|:-:|
| `0x00` - `0x09`| Configure PWM channel #0 to #9 for PWM | 8 | `struct PwmConfig;` | Configuration data for PWM channel |
| `0x60` - `0x69`| Configure PWM channel #0 to #9 for capture | 0 | - | - |

#### `PwmConfig`

| Bit(s) | Description |
|:-:|-|
| 0 | Enable/disable PWM output, i.e. `0` = output disabled |
| 1 | Polarity of PWM output, `1` = positive clock polarity = output active high |
| 2 - 32 | PWM Duty Cycle / ns |
| 33 - 64 | PWM Period / ns |
