#ifndef MODBUS_FIFO_INTERNAL_H
#define MODBUS_FIFO_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

uint8_t mb_fifo_process_read_request(const uint8_t *request_pdu,
                                     size_t request_pdu_len,
                                     uint8_t *response_pdu,
                                     size_t response_capacity,
                                     size_t *response_pdu_len);

#endif /* MODBUS_FIFO_INTERNAL_H */
