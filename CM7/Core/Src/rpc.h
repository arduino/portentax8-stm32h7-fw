#ifdef __cplusplus
extern "C" {
#endif

void serial_rpc_begin();
int serial_rpc_available();
void serial_rpc_read(uint8_t* buf);
void serial_rpc_write(uint8_t* buf, size_t len);

#ifdef __cplusplus
}
#endif