#ifndef MODBUS_RTU_SERVER_ID_INTERNAL_H
#define MODBUS_RTU_SERVER_ID_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

#define MBRTU_SERVER_ID_PDU_NOT_HANDLED 0
#define MBRTU_SERVER_ID_PDU_RESPONSE 1
#define MBRTU_SERVER_ID_PDU_ERROR (-1)
#define MBRTU_SERVER_ID_PDU_CAPACITY (-2)

int mbrtu_server_id_process_pdu(const uint8_t *request_pdu,
                                size_t request_pdu_len,
                                uint8_t *response_pdu,
                                size_t response_capacity,
                                size_t *response_pdu_len);

#endif /* MODBUS_RTU_SERVER_ID_INTERNAL_H */
