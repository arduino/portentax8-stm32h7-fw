#include "openamp.h"
#include "arduino_openamp.h"
#include "stm32h7xx_hal.h"
#include "ringbuffer.h"

enum endpoints_t {
	ENDPOINT_RAW = 0,
	ENDPOINT_RESPONSE
};

static struct rpmsg_endpoint rp_endpoints[4];
extern ring_buffer_t virtual_uart_ring_buffer;

int rpmsg_recv_raw_callback(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
	printf("raw: received %d bytes from M4, content:", len);
	for (int i = 0; i<len; i++) {
		printf("%x ", ((uint8_t*)data)[i]);
	}
	printf("\n");
	ring_buffer_queue_arr(&virtual_uart_ring_buffer, (uint8_t*)data, len);

	return 0;
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
	int idx = -1;

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
	OPENAMP_send(&rp_endpoints[0], buf, len);
}

void HSEM1_IRQHandler(void)
{
	HAL_HSEM_IRQHandler();
	OPENAMP_check_for_message();
}
