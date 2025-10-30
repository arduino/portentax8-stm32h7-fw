#include "double-buffer.h"
#include <stdint.h>

/* --------------------------------------------------------------------------*/
DblBuffer_t *dblBuffer_init(uint8_t *rx1, uint8_t *rx2, uint8_t *tx1, uint8_t *tx2, uint16_t lrx, uint16_t ltx) {
  DblBuffer_t *rv = (DblBuffer_t *)malloc(sizeof(DblBuffer_t));
  if(rv != NULL) {
    rv->len_rx         = lrx;
    rv->len_tx          = ltx;
    rv->current_pos_rx  = 0;
    rv->current_pos_tx  = 0;
    rv->rx_to_receive   = rx1;
    rv->rx_to_read      = rx2;
    rv->tx_to_send      = tx1;
    rv->tx_to_write     = tx2;
    memset(rx1,0x00,lrx);
    memset(rx2,0x00,lrx);
    memset(tx1,0x00,ltx);
    memset(tx2,0x00,ltx);
  }
  return rv;
}

/* --------------------------------------------------------------------------*/
void dblBuffer_destructor(DblBuffer_t *db) {
  if(db != NULL) {
    free(db);
    db = NULL;
  }
}

/* --------------------------------------------------------------------------*/
uint8_t *dblBuffer_getRXtoRead(DblBuffer_t *db) {
  if(db != NULL) {
    return db->rx_to_read;
  }
  return NULL;
}


void dblBuffer_increaseRXtoReceiveCurrentPos(DblBuffer_t *db, uint16_t offset) {
  if(db != NULL) {
    db->current_pos_rx += offset;
  }
}

/* --------------------------------------------------------------------------*/
uint8_t *dblBuffer_getRXtoReceive(DblBuffer_t *db) {
  if(db != NULL) {
    return db->rx_to_receive + db->current_pos_rx;
  }
  return NULL;
}

/* --------------------------------------------------------------------------*/
uint8_t *dblBuffer_getTXtoSend(DblBuffer_t *db) {
  if(db != NULL) {
    return db->tx_to_send;
  }
  return NULL;
}

uint8_t *dblBuffer_getTXtoWrite(DblBuffer_t *db) {
  if(db != NULL) {
    return db->tx_to_write;
  }
  return NULL;
}

/* --------------------------------------------------------------------------*/
void dblBuffer_swapTX(DblBuffer_t *db) {
  if(db != NULL) {
    void *tmp = db->tx_to_send;
    db->tx_to_send = db->tx_to_write;
    db->tx_to_write = tmp;
  }
}

/* --------------------------------------------------------------------------*/
void dblBuffer_swapRX(DblBuffer_t *db) {
  if(db != NULL) {
    void *tmp = db->rx_to_read;
    db->rx_to_read = db->rx_to_receive;
    db->rx_to_receive = tmp;
    db->current_pos_rx = 0;
  }
}

