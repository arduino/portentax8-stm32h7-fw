/*
 * Firmware for the Portenta X8 STM32H747AIIX/Cortex-M7 core.
 * Copyright (C) 2022 Arduino (http://www.arduino.cc/)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "openamp.h"
#include "arduino_openamp.h"
#include "stm32h7xx_hal.h"
#include "ringbuffer.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum endpoints_t {
  ENDPOINT_RAW = 0,
  ENDPOINT_RESPONSE
};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static struct rpmsg_endpoint rp_endpoints[4];
extern ring_buffer_t virtual_uart_ring_buffer;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int rpmsg_recv_raw_callback(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
/*
  printf("raw: received %d bytes from M4, content:", len);
  for (int i = 0; i<len; i++) {
    printf("%x ", ((uint8_t*)data)[i]);
  }
  printf("\n");
*/
  ring_buffer_queue_arr(&virtual_uart_ring_buffer, (const char *)data, len);

  return 0;
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
  if (strcmp(name, "raw") == 0) {
    OPENAMP_create_endpoint(&rp_endpoints[ENDPOINT_RAW], name, dest, rpmsg_recv_raw_callback, NULL);
  }
  if (strcmp(name, "response") == 0) {
    OPENAMP_create_endpoint(&rp_endpoints[ENDPOINT_RESPONSE], name, dest, rpmsg_recv_raw_callback, NULL);
  }
}

int serial_rpc_begin() {

  /* Initialize OpenAmp and libmetal libraries */
  if (MX_OPENAMP_Init(RPMSG_MASTER, new_service_cb) !=  HAL_OK) {
    return 0;
  }

  /* Initialize the rpmsg endpoint to set default addresses to RPMSG_ADDR_ANY */
  rpmsg_init_ept(&rp_endpoints[0], "raw", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);
  rpmsg_init_ept(&rp_endpoints[1], "response", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);

  /*
  * The rpmsg service is initiate by the remote processor, on H7 new_service_cb
  * callback is received on service creation. Wait for the callback
  */
  OPENAMP_Wait_EndPointready(&rp_endpoints[0], HAL_GetTick() + 500);
  OPENAMP_Wait_EndPointready(&rp_endpoints[1], HAL_GetTick() + 500);

  // Send first dummy message to enable the channel
  int message = 0x00;
  OPENAMP_send(&rp_endpoints[0], &message, sizeof(message));
  OPENAMP_send(&rp_endpoints[1], &message, sizeof(message));

  return 1;
}

int serial_rpc_available() {
  OPENAMP_check_for_message();
}

void serial_rpc_write(uint8_t* buf, size_t len) {
  // check second byte of the message to split requests and responses
  OPENAMP_send(&rp_endpoints[buf[1] == 1], buf, len);
}

void HSEM1_IRQHandler(void)
{
  HAL_HSEM_IRQHandler();
  OPENAMP_check_for_message();
}
