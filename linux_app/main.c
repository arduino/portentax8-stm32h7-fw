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

#if !defined(__linux__)

static port_err_t spi_open(struct port_interface __unused *port,
			   struct port_options __unused *ops)
{
	return PORT_ERR_NODEV;
}

struct port_interface port_spi = {
	.name	= "spi",
	.open	= spi_open,
};

#else

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

struct spi_priv {
	int fd;
	uint8_t mode, lsb, bits;
  uint32_t speed;
};

__attribute__((packed, aligned(4))) struct subpacket {
  uint8_t peripheral;
  uint8_t opcode;
  uint16_t size;
  uint8_t raw_data;
};

__attribute__((packed, aligned(4))) struct complete_packet {
  uint16_t size;
  struct subpacket data;
  // ... other subpackets will follow
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

int need_sync = true;

ssize_t spi_transfer(int fd, void *out, void *in, size_t len)
{
    struct spi_ioc_transfer msgs[2] = {};

    memset(msgs, 0, sizeof(msgs));

  	uint8_t buf[] = { 0x5A, 0x00, 0x79 };
  	uint8_t temp_buf[len + 1];

  	msgs[0].tx_buf = out;
  	msgs[0].rx_buf = temp_buf;
  	msgs[0].len = len;
  	msgs[0].cs_change = 0;

  	msgs[1].tx_buf = out;
  	msgs[1].rx_buf = in ? in : temp_buf;
  	msgs[1].len = len;
  	msgs[1].cs_change = 0;

  	if(ioctl(fd, SPI_IOC_MESSAGE(1), &msgs) < 0) {
  		printf("ioctl failed: %d\n", len);
      return PORT_ERR_UNKNOWN;
    }

    if (in) {

    	memmove(in, temp_buf, len);

/*
    	if (((uint8_t*)in)[0] == 0xA5) {
    		return PORT_ERR_TIMEDOUT;
    	} else {
    		if ((((uint8_t*)in)[0] == 0x79)) {
    			msgs[0].tx_buf = &buf[2];
    			msgs[0].rx_buf = temp_buf;
    			msgs[0].len = 1;
    			if(ioctl(fd, SPI_IOC_MESSAGE(1), &msgs) < 0)
        		return PORT_ERR_UNKNOWN;
    		} else {
    			printf("got %x\n", ((uint8_t*)in)[0]);
    		}
    	}
    */
    }
/*
    if (in) {
    	memcpy(in, &temp_buf[1], len);
    }
*/

    return len;
}

static port_err_t spi_read(struct port_interface *port, void *buf,
			   size_t nbyte)
{
	struct spi_priv *h;
	int ret;

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;
	ret = spi_transfer(h->fd, NULL, buf, nbyte);
	if (ret != (int)nbyte)
		return ret;
	return PORT_ERR_OK;
}

static port_err_t spi_write(struct port_interface *port, void *buf,
			    size_t nbyte)
{
	struct spi_priv *h;
	int ret;

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;
	ret = spi_transfer(h->fd, buf, NULL, nbyte);

	if (ret != (int)nbyte)
		return ret;
	return PORT_ERR_OK;
}

static port_err_t spi_gpio(struct port_interface *port, int level)
{
	return PORT_ERR_OK;
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

static port_err_t spi_flush(struct port_interface *port)
{
	/* We shouldn't need to flush I2C */
	return PORT_ERR_OK;
}

static struct varlen_cmd spi_cmd_get_reply[] = {
	{0x10, 11},
	{0x11, 13},
	{0x12, 18},
	{ /* sentinel */ }
};

#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

int main(int argc, char** argv) {

	struct port_interface port;
	struct port_options ops;

	ops.device = "/dev/spidev1.1";

	spi_open(&port, &ops);

	printf(spi_get_cfg_str(&port));

	uint8_t samplebuffer[50] = { 
		19,		// lsb_len
		0,		// msb_len
		0x12,	// peripheral
		0x34,	// opcode
		0x01,	// data_len_lsb
		0x00,	// data_len_msb
		0x77,	// data
		0x99,	// peripheral
		0xAB,	// opcode
		0x04,	// data_len_lsb
		0x00,	// data_len_msb
		0x87,	// data
		0x88,	// data
		0x89,	// data
		0x90,	// data
		0x03,	// peripheral
		0x02,	// opcode
		0x01,	// data_len_lsb
		0x00,	// data_len_msb
		0x11,	// data
	};

	uint8_t rxb[50];
	memset(rxb, 0, sizeof(rxb));

	struct subpacket *rx_pkt_userspace = (struct subpacket *)rxb;

	uint16_t rx_data;

	struct spi_priv * h = (struct spi_priv *)port.private;

	spi_transfer(h->fd, samplebuffer, &rx_data, sizeof(uint16_t));
	printf("STM32 has %d bytes\n", rx_data);
	printf("Sending %d bytes\n", max(((uint16_t)samplebuffer[1] << 8 | samplebuffer[0]), rx_data));
	spi_transfer(h->fd, &samplebuffer[2], &rxb, max(((uint16_t)samplebuffer[1] << 8 | samplebuffer[0]), rx_data));
	printf("Input data: %x %x %x %x %x %x %x\n", rxb[0], rxb[1], rxb[2], rxb[3], rxb[4], rxb[5], rxb[6]);

	while (rx_pkt_userspace->peripheral != 0xFF && rx_pkt_userspace->peripheral != 0x00) {
	    printf("Peripheral: %X Opcode: %X Size: %X\n  data: ",
	            rx_pkt_userspace->peripheral, rx_pkt_userspace->opcode,
	            rx_pkt_userspace->size);
	    for (int i = 0; i < rx_pkt_userspace->size; i++) {
	      printf("0x%02X ", *((uint8_t*)(((uint8_t*)(&rx_pkt_userspace->raw_data)) + i)));
	    }
	    printf("\n");
	    rx_pkt_userspace =
	        (uint8_t *)rx_pkt_userspace + 4 + rx_pkt_userspace->size;
	}

	spi_close(&port);

	return 0;
}

struct port_interface port_spi = {
	.name	= "spi",
	.flags	= PORT_CMD_INIT | PORT_RETRY | PORT_GVR_ETX | PORT_PROTOCOL_SPI,
	.open	= spi_open,
	.close	= spi_close,
	.flush  = spi_flush,
	.read	= spi_read,
	.write	= spi_write,
	.gpio	= spi_gpio,
	.get_cfg_str	= spi_get_cfg_str,
	.cmd_get_reply	= spi_cmd_get_reply,
};

#endif

