#ifndef PORTENTA_X8_DOUBLE_BUFFER
#define PORTENTA_X8_DOBULE_BUFFER

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef struct dblBuffer {
  uint16_t len_rx;
  uint16_t len_tx;
  uint16_t current_pos_rx;
  uint16_t current_pos_tx;
  void *rx_to_read;
  void *rx_to_receive;
  void *tx_to_write;
  void *tx_to_send;

} DblBuffer_t;

DblBuffer_t *dblBuffer_init(uint8_t *rx1, uint8_t *rx2, uint8_t *tx1, uint8_t *tx2, uint16_t lrx, uint16_t ltx);
void dblBuffer_destructor(DblBuffer_t *db);
/* for the application to read from */
uint8_t *dblBuffer_getRXtoRead(DblBuffer_t *db);
/* for the peripheral to put data into */
uint8_t *dblBuffer_getRXtoReceive(DblBuffer_t *db);
/* for the peripheral to send */
uint8_t *dblBuffer_getTXtoSend(DblBuffer_t *db);
/* for the application to write into */
uint8_t *dblBuffer_getTXtoWrite(DblBuffer_t *db);
void dblBuffer_swapTX(DblBuffer_t *db);
void dblBuffer_swapRX(DblBuffer_t *db);

void dblBuffer_increaseRXtoReceiveCurrentPos(DblBuffer_t *db, uint16_t offset);


#endif
