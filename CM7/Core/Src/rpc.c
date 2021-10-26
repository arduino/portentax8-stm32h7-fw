#include "openamp.h"
#include "arduino_openamp.h"
#include "stm32h7xx_hal.h"
#include "ringbuffer.h"

enum endpoints_t {
	ENDPOINT_CM7TOCM4 = 0,
	ENDPOINT_CM4TOCM7,
	ENDPOINT_RAW
};

static struct rpmsg_endpoint rp_endpoints[4];
extern ring_buffer_t virtual_uart_ring_buffer;

int rpmsg_recv_cm4tocm7_callback(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
/*
	printf("4to7: received %d bytes from M4, content:", len);
	for (int i = 0; i<len; i++) {
		printf("%x ", ((uint8_t*)data)[i]);
	}
	printf("\n");
*/
	ring_buffer_queue_arr(&virtual_uart_ring_buffer, (uint8_t*)data, len);
	OPENAMP_send(ept, data, len);
	return 0;
}

int rpmsg_recv_cm7tocm4_callback(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
/*
	printf("7to4: received %d bytes from M4, content:", len);
	for (int i = 0; i<len; i++) {
		printf("%x ", ((uint8_t*)data)[i]);
	}
	printf("\n");
*/
	OPENAMP_send(ept, data, len);
	return 0;
}

int rpmsg_recv_raw_callback(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
	printf("raw: received %d bytes from M4, content:", len);
	for (int i = 0; i<len; i++) {
		printf("%x ", ((uint8_t*)data)[i]);
	}
	printf("\n");
	return 0;
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
	int idx = -1;

	if (strcmp(name, "cm7tocm4") == 0) {
		OPENAMP_create_endpoint(&rp_endpoints[ENDPOINT_CM7TOCM4], name, dest, rpmsg_recv_cm7tocm4_callback, NULL);
	}
	if (strcmp(name, "cm4tocm7") == 0) {
		OPENAMP_create_endpoint(&rp_endpoints[ENDPOINT_CM4TOCM7], name, dest, rpmsg_recv_cm4tocm7_callback, NULL);
	}
	if (strcmp(name, "raw") == 0) {
		OPENAMP_create_endpoint(&rp_endpoints[ENDPOINT_RAW], name, dest, rpmsg_recv_raw_callback, NULL);
	}
}

int serial_rpc_begin() {

	/* Initialize OpenAmp and libmetal libraries */
	if (MX_OPENAMP_Init(RPMSG_MASTER, new_service_cb) !=  HAL_OK) {
		return 0;
	}

	/* Initialize the rpmsg endpoint to set default addresses to RPMSG_ADDR_ANY */
	rpmsg_init_ept(&rp_endpoints[0], "cm7tocm4", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);
	rpmsg_init_ept(&rp_endpoints[1], "cm4tocm7", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);
	rpmsg_init_ept(&rp_endpoints[2], "raw", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);

	/*
	* The rpmsg service is initiate by the remote processor, on H7 new_service_cb
	* callback is received on service creation. Wait for the callback
	*/
	OPENAMP_Wait_EndPointready(&rp_endpoints[0], HAL_GetTick() + 500);
	OPENAMP_Wait_EndPointready(&rp_endpoints[1], HAL_GetTick() + 500);
	OPENAMP_Wait_EndPointready(&rp_endpoints[2], HAL_GetTick() + 500);

	// Send first dummy message to enable the channel
	int message = 0x00;
	OPENAMP_send(&rp_endpoints[0], &message, sizeof(message));
	OPENAMP_send(&rp_endpoints[1], &message, sizeof(message));
	OPENAMP_send(&rp_endpoints[2], &message, sizeof(message));

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
