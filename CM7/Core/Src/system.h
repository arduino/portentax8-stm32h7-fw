#include <inttypes.h>
#include <stdbool.h>

#ifndef SYSTEM_H
#define SYSTEM_H

#define SPI_DMA_BUFFER_SIZE 	2048

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

#define max(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a > _b ? _a : _b;                                                         \
  })

void system_init();

void dma_init();

void enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void* data);

void dispatchPacket(uint8_t peripheral, uint8_t opcode, uint16_t size, uint8_t* data);

struct complete_packet* get_dma_packet();

int get_dma_packet_size();

void dma_handle_data();

void register_peripheral_callback(uint8_t peripheral,/* uint8_t opcode,*/ void* func);

#endif //SYSTEM_H