#include "modbus_rtu_server_id.h"

#include "modbus_rtu_server_id_internal.h"
#include "platform_port.h"

#include <string.h>

typedef struct {
    uint8_t configured;
    size_t byte_count;
    uint8_t data[MBRTU_SERVER_ID_MAX_BYTE_COUNT];
} mbrtu_server_id_state_t;

static mbrtu_server_id_state_t server_id_state;

int mbrtu_server_id_configure(const uint8_t *server_id,
                              size_t server_id_length,
                              uint8_t run_status,
                              const uint8_t *additional_data,
                              size_t additional_data_length)
{
    size_t byte_count;

    if (server_id == NULL ||
        (additional_data_length > 0u && additional_data == NULL)) {
        return MBRTU_SERVER_ID_ERROR_ARGUMENT;
    }
    if (server_id_length == 0u ||
        server_id_length > MBRTU_SERVER_ID_MAX_SERVER_ID_LENGTH) {
        return MBRTU_SERVER_ID_ERROR_LENGTH;
    }
    if (run_status != MBRTU_SERVER_ID_RUN_STATUS_OFF &&
        run_status != MBRTU_SERVER_ID_RUN_STATUS_ON) {
        return MBRTU_SERVER_ID_ERROR_STATUS;
    }
    if (additional_data_length > MBRTU_SERVER_ID_MAX_ADDITIONAL_DATA) {
        return MBRTU_SERVER_ID_ERROR_CAPACITY;
    }

    byte_count = server_id_length + 1u + additional_data_length;
    if (byte_count > MBRTU_SERVER_ID_MAX_BYTE_COUNT) {
        return MBRTU_SERVER_ID_ERROR_CAPACITY;
    }

    mb_lock();
    memset(&server_id_state, 0, sizeof(server_id_state));
    memcpy(server_id_state.data, server_id, server_id_length);
    server_id_state.data[server_id_length] = run_status;
    if (additional_data_length > 0u) {
        memcpy(&server_id_state.data[server_id_length + 1u],
               additional_data,
               additional_data_length);
    }
    server_id_state.byte_count = byte_count;
    server_id_state.configured = 1u;
    mb_unlock();
    return MBRTU_SERVER_ID_OK;
}

void mbrtu_server_id_clear(void)
{
    mb_lock();
    memset(&server_id_state, 0, sizeof(server_id_state));
    mb_unlock();
}

int mbrtu_server_id_is_configured(void)
{
    int configured;

    mb_lock();
    configured = server_id_state.configured != 0u;
    mb_unlock();
    return configured;
}

static void write_exception(uint8_t *response_pdu,
                            size_t *response_pdu_len,
                            uint8_t exception_code)
{
    response_pdu[0] = (uint8_t)(MBRTU_SERVER_ID_FUNCTION_CODE | 0x80u);
    response_pdu[1] = exception_code;
    *response_pdu_len = 2u;
}

int mbrtu_server_id_process_pdu(const uint8_t *request_pdu,
                                size_t request_pdu_len,
                                uint8_t *response_pdu,
                                size_t response_capacity,
                                size_t *response_pdu_len)
{
    size_t response_length;

    if (request_pdu == NULL || response_pdu == NULL ||
        response_pdu_len == NULL) {
        return MBRTU_SERVER_ID_PDU_ERROR;
    }
    *response_pdu_len = 0u;

    if (request_pdu_len == 0u ||
        request_pdu[0] != MBRTU_SERVER_ID_FUNCTION_CODE) {
        return MBRTU_SERVER_ID_PDU_NOT_HANDLED;
    }
    if (response_capacity < 2u) {
        return MBRTU_SERVER_ID_PDU_CAPACITY;
    }
    if (request_pdu_len != 1u) {
        write_exception(response_pdu, response_pdu_len, 0x03u);
        return MBRTU_SERVER_ID_PDU_RESPONSE;
    }

    mb_lock();
    if (server_id_state.configured == 0u) {
        mb_unlock();
        write_exception(response_pdu, response_pdu_len, 0x04u);
        return MBRTU_SERVER_ID_PDU_RESPONSE;
    }

    response_length = 2u + server_id_state.byte_count;
    if (response_capacity < response_length) {
        mb_unlock();
        return MBRTU_SERVER_ID_PDU_CAPACITY;
    }

    response_pdu[0] = MBRTU_SERVER_ID_FUNCTION_CODE;
    response_pdu[1] = (uint8_t)server_id_state.byte_count;
    memcpy(&response_pdu[2],
           server_id_state.data,
           server_id_state.byte_count);
    mb_unlock();

    *response_pdu_len = response_length;
    return MBRTU_SERVER_ID_PDU_RESPONSE;
}
