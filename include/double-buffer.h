#ifndef PORTENTA_X8_DOUBLE_BUFFER
#define PORTENTA_X8_DOBULE_BUFFER

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef struct dblBuffer {
  uint16_t len_rx;
  uint16_t len_tx;
  uint16_t current_pos_rx;
  uint16_t pos_tx_when_writing;
  uint16_t pos_tx_when_removing;
  uint16_t tx_to_be_removed;
  void *rx_to_read;
  void *rx_to_receive;
  void *tx_to_write;
  void *tx_to_send;

} DblBuffer_t;

void dblBuffer_init(DblBuffer_t *db, uint8_t *rx1, uint8_t *rx2, uint8_t *tx1, uint8_t *tx2, uint16_t lrx, uint16_t ltx);
void dblBuffer_destructor(DblBuffer_t *db);
/* for the application to read from */
uint8_t *dblBuffer_getRXtoRead(DblBuffer_t *db);
/* for the peripheral to put data into */
uint8_t *dblBuffer_getRXtoReceive(DblBuffer_t *db);
/* for the peripheral to send */
uint8_t *dblBuffer_getTXtoSend(DblBuffer_t *db,uint8_t flag_with_offset);
/* for the application to write into 
 * if the flag is != from 0 then the pointer is returned to the current 
 * available position
 * if the flag is 0 then the is returned the pointer to beginning of the buffer*/
uint8_t *dblBuffer_getTXtoWrite(DblBuffer_t *db, uint8_t flag_with_offset);
void dblBuffer_swapTX(DblBuffer_t *db);
void dblBuffer_swapRX(DblBuffer_t *db);

void dblBuffer_increaseRXtoReceiveCurrentPos(DblBuffer_t *db, uint16_t offset);
void dblBuffer_increaseTXtoWritePosWhenWriting(DblBuffer_t *db, uint16_t offset);
void dblBuffer_increaseTXtoWritePosWhenRemoving(DblBuffer_t *db, uint16_t offset);
uint16_t dblBuffer_getTXtoWriteWhenWriting(DblBuffer_t *db);
uint16_t dblBuffer_getTXtoWriteWhenRemoving(DblBuffer_t *db);
uint16_t dblBuffer_howMuchTXtoWrite(DblBuffer_t *db);
int dblBuffer_numTxToRemove(DblBuffer_t *db);

#define DBL_BUFF_UART_SIZE 4096

#endif
