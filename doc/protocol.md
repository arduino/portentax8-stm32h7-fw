X8/H7 Protocol
==============
## SPI
The data exchanged between the IMX8 and H7 has the layout of one superframe which can consist of zero, one or multiple subframes. The purpose behind this design decision it to enqueue multiple IO requests/responses withing a single SPI transfer in order to reduce the overall amount if SPI transfers happening.
### Superframe Description
| Name | Byte | Size / Bytes | Description |
|:-:|:-:|:-:|-|
| `size` | 0-1 | 2 | Total superframe length in bytes (max. 2048 bytes) |
| `checksum` | 2-3 | 2 | Checksum calculated by xor-ing super frame length with `0x5555`. |
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
