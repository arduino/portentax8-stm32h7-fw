#include "double-buffer.h"
#include <stdint.h>
#include <stdlib.h> // For NULL
#include <string.h> // For memset

/* --------------------------------------------------------------------------*/
// MODIFIED: No longer returns a pointer, just initializes the one passed in.
void dblBuffer_init(DblBuffer_t *db, uint8_t *rx1, uint8_t *rx2, uint8_t *tx1, uint8_t *tx2, uint16_t lrx, uint16_t ltx) {
  /* No malloc. The caller must provide a valid db pointer. */
  if(db == NULL) {
    return;
  }
  
  db->len_rx               = lrx;
  db->len_tx               = ltx;
  db->current_pos_rx       = 0;
  db->pos_tx_when_writing  = 0;
  db->rx_to_receive        = rx1;
  db->rx_to_read           = rx2;
  db->tx_to_send           = tx1;
  db->tx_to_write          = tx2;
  db->pos_tx_when_removing = 0;
  db->tx_to_be_removed     = 0;
  
  if(rx1 != NULL)
    memset(rx1,0x00,lrx);
  if(rx2 != NULL)
    memset(rx2,0x00,lrx);
  if(tx1 != NULL)
    memset(tx1,0x00,ltx);
  if(tx2 != NULL)
    memset(tx2,0x00,ltx);
}


/* --------------------------------------------------------------------------*/
void dblBuffer_reset(DblBuffer_t *db) {
  if(db != NULL) {
    if(db->rx_to_receive != NULL)
      memset(db->rx_to_receive,0x00,db->len_rx);
    if(db->rx_to_read != NULL)
      memset(db->rx_to_read,0x00,db->len_rx);
    if(db->tx_to_write != NULL)
      memset(db->tx_to_write,0x00,db->len_tx);
    if(db->tx_to_send != NULL)
      memset(db->tx_to_send,0x00,db->len_tx);
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

void dblBuffer_increaseTXtoWritePosWhenWriting(DblBuffer_t *db, uint16_t offset) {
  if(db != NULL) {
    db->pos_tx_when_writing += offset;
  }
}

void dblBuffer_increaseTXtoWritePosWhenRemoving(DblBuffer_t *db, uint16_t offset) {
  if(db != NULL) {
    db->pos_tx_when_removing += offset;
  }
}

uint16_t dblBuffer_getTXtoWriteWhenWriting(DblBuffer_t *db) {
  if(db != NULL) {
    return db->pos_tx_when_writing;
  }
  return 0;
}

uint16_t dblBuffer_getTXtoWriteWhenRemoving(DblBuffer_t *db) {
  if(db != NULL) {
    return db->pos_tx_when_removing;
  }
  return 0;
}

uint16_t dblBuffer_howMuchTXtoWrite(DblBuffer_t *db) {
  if(db != NULL) {
    return db->tx_to_be_removed;
  }
  return 0;

}

/* --------------------------------------------------------------------------*/
uint8_t *dblBuffer_getRXtoReceive(DblBuffer_t *db) {
  if(db != NULL) {
    return db->rx_to_receive + db->current_pos_rx;
  }
  return NULL;
}

/* --------------------------------------------------------------------------*/
uint8_t *dblBuffer_getTXtoSend(DblBuffer_t *db, uint8_t flag_with_offset) {
  if(db != NULL) {
    if(flag_with_offset != 0) {
      return  db->tx_to_send + db->pos_tx_when_removing;
    }
    return db->tx_to_send;
  }
  return NULL;
}

uint8_t *dblBuffer_getTXtoWrite(DblBuffer_t *db, uint8_t flag_with_offset) {
  if(db != NULL) {
    if(flag_with_offset != 0) {
      return db->tx_to_write + db->pos_tx_when_writing;
    } 
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
    /* save the position */
    db->tx_to_be_removed = db->pos_tx_when_writing;
    db->pos_tx_when_removing = 0;
    db->pos_tx_when_writing = 0; 
  }
}

int dblBuffer_numTxToRemove(DblBuffer_t *db) {
  if(db != NULL) {
    return ((int)db->tx_to_be_removed - (int)db->pos_tx_when_removing);
  }
  return 0;
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

