#ifndef MODBUS_DEVICE_ID_INTERNAL_H
#define MODBUS_DEVICE_ID_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

uint8_t mb_device_id_process_request(uint8_t read_device_id_code,
                                     uint8_t requested_object_id,
                                     uint8_t *response_pdu,
                                     size_t response_capacity,
                                     size_t *response_pdu_len);

#endif /* MODBUS_DEVICE_ID_INTERNAL_H */
