#include "RPC_internal.h"

void serial_rpc_begin() {
	SerialRPC.begin();
}

int serial_rpc_available() {
	return SerialRPC.available();
}

void serial_rpc_read(uint8_t* buf) {
	int available = SerialRPC.available();
	for (int i = 0; i < available; i++) {
		buf[i] = SerialRPC.read();
	}
}

void serial_rpc_write(uint8_t* buf, size_t len) {
	SerialRPC.write(buf, len);
}