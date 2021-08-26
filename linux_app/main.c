/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2014 Antonio Borneo <borneo.antonio@gmail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdbool.h>

#include "port.h"

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

struct spi_priv {
  int fd;
  uint8_t mode, lsb, bits;
  uint32_t speed;
};

static port_err_t spi_open(struct port_interface *port,
			   struct port_options *ops)
{
	struct spi_priv *h;
	int fd, ret;
	unsigned long funcs;
	uint8_t mode = 0;
	uint8_t lsb = 0;
	uint8_t bits;
  uint32_t speed = 1000000;

	/* 1. check device name match */
	if (strncmp(ops->device, "/dev/spidev", strlen("/dev/spidev")))
		return PORT_ERR_NODEV;

	/* 2. open it */
	h = calloc(sizeof(*h), 1);
	if (h == NULL) {
		fprintf(stderr, "End of memory\n");
		return PORT_ERR_UNKNOWN;
	}
	fd = open(ops->device, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Unable to open special file \"%s\"\n",
			ops->device);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	/* 3.5. Check capabilities */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_MODE) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_MODE) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_MODE) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_MODE) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_BIT_PER_WORD) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(WRITE_SPEED) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	h->fd = fd;
	h->mode = mode;
	h->lsb = lsb;
	h->bits = bits;
	h->speed = speed;
	port->private = h;
	return PORT_ERR_OK;
}

static port_err_t spi_close(struct port_interface *port)
{
	struct spi_priv *h;

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;
	close(h->fd);
	free(h);
	port->private = NULL;
	return PORT_ERR_OK;
}

static ssize_t spi_transfer(int fd, void *out, void *in, size_t len)
{
    struct spi_ioc_transfer msgs[1] = {};

    memset(msgs, 0, sizeof(msgs));

    msgs[0].tx_buf = (uint64_t)out;
    msgs[0].rx_buf = (uint64_t)in;
    msgs[0].len = len;

  	if(ioctl(fd, SPI_IOC_MESSAGE(1), &msgs) < 0) {
  		printf("ioctl failed: %d\n", len);
      return PORT_ERR_UNKNOWN;
    }

    return len;
}

static const char *spi_get_cfg_str(struct port_interface *port)
{
	struct spi_priv *h;
	static char str[50];

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return "INVALID";
	snprintf(str, sizeof(str), "mode:%d lsb:%d bits:%d speed:%lu\n", h->mode, h->lsb, h->bits, h->speed);
	return str;
}


__attribute__((packed, aligned(4))) struct subpacket {
  uint8_t peripheral;
  uint8_t opcode;
  uint16_t size;
  uint8_t raw_data;
};

__attribute__((packed, aligned(4))) struct complete_packet {
  uint16_t size;
  uint16_t checksum;
  struct subpacket data;
  // ... other subpackets will follow
};

enum Peripherals {
	PERIPH_ADC = 0x01,
	PERIPH_PWM = 0x02,
	PERIPH_FDCAN1 = 0x03,
	PERIPH_FDCAN2 = 0x04,
	PERIPH_UART = 0x05,
	PERIPH_RTC = 0x06,
	PERIPH_GPIO = 0x07,
	PERIPH_MAX,
};

const char* to_peripheral_string(enum Peripherals peripheral) {
	switch (peripheral) {
		case PERIPH_ADC:
			return "ADC";
		case PERIPH_PWM:
			return "PWM";
		case PERIPH_FDCAN1:
			return "FDCAN1";
		case PERIPH_FDCAN2:
			return "FDCAN2";
		case PERIPH_UART:
			return "UART";
		case PERIPH_RTC:
			return "RTC";
		case PERIPH_GPIO:
			return "GPIO";
		default:
			return "UNKNOWN";
	}
}

enum Opcodes {
	CONFIGURE = 0x10,
	DATA = 0x01,
};

enum Opcodes_UART {
	GET_LINESTATE = 0x20,
};

enum Opcodes_RTC {
	SET_DATE = 0x01,
	GET_DATE = 0x02,
	SET_ALARM = 0x11,
	GET_ALARM = 0x12,
};

enum Opcodes_GPIO {
	DIRECTION = 0x10,
	WRITE = 0x20,
	READ = 0x30,
};

enum AnalogPins {
	A0 = 0x1,
	A1,
	A2,
	A3,
	A4,
	A5,
	A6,
	A7,
};

struct __attribute__((packed, aligned(4))) pwmPacket {
	uint8_t enable: 1;
	uint8_t polarity: 1;
	uint16_t duty: 10;
	uint32_t frequency: 20;
};

uint8_t samplebuffer[UINT16_MAX];

void enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void* data) {

	struct complete_packet *tx_pkt = (struct complete_packet *)samplebuffer;
	uint16_t offset = tx_pkt->size;
	if (offset + size > sizeof(samplebuffer)) {
		return;
	}
	struct subpacket pkt;
	pkt.peripheral = peripheral;
	pkt.opcode = opcode;
	pkt.size = size;
	memcpy((uint8_t*)&(tx_pkt->data) + offset, &pkt, 4);
	memcpy((uint8_t*)&(tx_pkt->data) + offset + 4, data, size);
	tx_pkt->size += 4 + size;
	tx_pkt->checksum = tx_pkt->size ^ 0x5555;
}

void dispatchPacket(uint8_t peripheral, uint8_t opcode, uint16_t size, uint8_t* data) {
	switch (peripheral) {
	case PERIPH_ADC:
		if (opcode != CONFIGURE) {
			printf("ADC%d: %d\n", opcode - 1, *((uint16_t*)data));
		}
		break;
  case PERIPH_FDCAN1:
    if (opcode == DATA) {
      printf("FDCAN1: %s\n", data);
    }
    break;
  }
}

#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

void print_packet_header(struct subpacket *pkt) {

	printf("Peripheral: %X Opcode: %X Size: %X\n  data: ",
		            pkt->peripheral, pkt->opcode,
		            pkt->size);

	if (pkt->size > 30) {
		printf("Not printing data\n");
		return;
	}

	for (int i = 0; i < pkt->size; i++) {
		printf("0x%02X ", *((uint8_t*)(((uint8_t*)(&pkt->raw_data)) + i)));
	}
	printf("\n");
}

void configureADCSampleRate(uint16_t rate) {
	enqueue_packet(PERIPH_ADC, CONFIGURE, sizeof(uint16_t), &rate);
}

void configurePWM(uint8_t channel, bool enable, bool polarity, float duty, uint32_t frequency) {
	struct pwmPacket conf;
	conf.enable = enable;
	conf.polarity = polarity;
	conf.duty = duty * 1024;
	conf.frequency = frequency;
	enqueue_packet(PERIPH_PWM, channel, sizeof(conf), &conf);
}

int main(int argc, char** argv) {

	struct port_interface port;
	struct port_options ops;

	ops.device = "/dev/spidev1.1";

	spi_open(&port, &ops);

	printf(spi_get_cfg_str(&port));

	memset(samplebuffer, 0 , sizeof(samplebuffer));

	configureADCSampleRate(100);
	configurePWM(2, true, false, 0.3356, 500000);

	uint8_t rxb[512];
	uint16_t rx_data[2];

	struct spi_priv * h = (struct spi_priv *)port.private;

	while (1) {

		spi_transfer(h->fd, samplebuffer, rx_data, sizeof(uint16_t) * 2);
		printf("STM32 has %d bytes\n", rx_data[0]);
		if (rx_data[0] != 0 && ((rx_data[0] ^ 0x5555) != rx_data[1])) {
			printf("Out of sync %x %x\n", rx_data[0], rx_data[1]);
			usleep(1000000);
			continue;
		}

		if (max(((uint16_t)samplebuffer[1] << 8 | samplebuffer[0]), rx_data[0]) == 0) {
			// send some nonsense anyway
			spi_transfer(h->fd, &samplebuffer[4], &rxb, 4);
			usleep(1000000);
			continue;
		}

		printf("Transferring %d bytes\n", max(((uint16_t)samplebuffer[1] << 8 | samplebuffer[0]), rx_data[0]));

		//memset(rxb, 0, sizeof(rxb));
		//usleep(500);

		spi_transfer(h->fd, &samplebuffer[4], &rxb, max(((uint16_t)samplebuffer[1] << 8 | samplebuffer[0]), rx_data[0]));

		struct subpacket *rx_pkt_userspace = (struct subpacket *)rxb;
		rxb[rx_data[0]] = 0xFF;

		while (rx_pkt_userspace->peripheral != 0xFF && rx_pkt_userspace->peripheral != 0x00 && rx_pkt_userspace->peripheral < PERIPH_MAX) {
		    print_packet_header(rx_pkt_userspace);

		    dispatchPacket(rx_pkt_userspace->peripheral, rx_pkt_userspace->opcode,
        		rx_pkt_userspace->size, (uint8_t*)(&rx_pkt_userspace->raw_data));

		    rx_pkt_userspace = (struct subpacket *)((uint8_t *)rx_pkt_userspace + 4 + rx_pkt_userspace->size);
		}

		// don't send anything after config phase
		memset(samplebuffer, 0, sizeof(samplebuffer));

		// FIXMEEEE: if I set 0 here, the whole stuff breaks
		// Probably, the STM DMA is one step "beyond" the actual execution, so the 2 step procedure creates a
		// DMA request which is "empty" for the iMX pow but contains some data from the STM
		// samplebuffer[0] = 50;

		usleep(1000000);
	}

	spi_close(&port);

	return 0;
}

struct port_interface port_spi = {
	.name	= "spi",
	.open	= spi_open,
	.close	= spi_close,
	.get_cfg_str	= spi_get_cfg_str,
};
