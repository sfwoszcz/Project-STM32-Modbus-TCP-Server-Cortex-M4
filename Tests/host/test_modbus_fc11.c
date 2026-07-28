#include "modbus_crc16.h"
#include "modbus_protocol.h"
#include "modbus_rtu.h"
#include "modbus_rtu_master.h"
#include "modbus_rtu_master_transaction.h"
#include "modbus_rtu_server_id.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

typedef struct {
    unsigned calls;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length;
} fake_transport_t;

static size_t append_crc(uint8_t *adu, size_t length_without_crc)
{
    uint16_t crc = mb_crc16(adu, length_without_crc);

    adu[length_without_crc] = (uint8_t)crc;
    adu[length_without_crc + 1u] = (uint8_t)(crc >> 8u);
    return length_without_crc + MODBUS_RTU_CRC_SIZE;
}

static void replace_crc(uint8_t *adu, size_t adu_length)
{
    uint16_t crc = mb_crc16(adu, adu_length - MODBUS_RTU_CRC_SIZE);

    adu[adu_length - 2u] = (uint8_t)crc;
    adu[adu_length - 1u] = (uint8_t)(crc >> 8u);
}

static size_t make_request(uint8_t address,
                           const uint8_t *extra,
                           size_t extra_length,
                           uint8_t *adu)
{
    adu[0] = address;
    adu[1] = MBRTU_SERVER_ID_FUNCTION_CODE;
    if (extra_length > 0u) {
        memcpy(&adu[2], extra, extra_length);
    }
    return append_crc(adu, 2u + extra_length);
}

static size_t make_response(uint8_t address,
                            const uint8_t *server_id,
                            size_t server_id_length,
                            uint8_t run_status,
                            const uint8_t *additional_data,
                            size_t additional_data_length,
                            uint8_t *adu)
{
    size_t byte_count =
        server_id_length + 1u + additional_data_length;

    adu[0] = address;
    adu[1] = MBRTU_SERVER_ID_FUNCTION_CODE;
    adu[2] = (uint8_t)byte_count;
    memcpy(&adu[3], server_id, server_id_length);
    adu[3u + server_id_length] = run_status;
    if (additional_data_length > 0u) {
        memcpy(&adu[4u + server_id_length],
               additional_data,
               additional_data_length);
    }
    return append_crc(adu, 3u + byte_count);
}

static int fake_transmit(void *context, const uint8_t *adu, size_t length)
{
    fake_transport_t *transport = (fake_transport_t *)context;

    ++transport->calls;
    transport->adu_length = length;
    memcpy(transport->adu, adu, length);
    return MBRTUM_TXN_TRANSMIT_ACCEPTED;
}

static int test_configuration(void)
{
    uint8_t mutable_server_id[] = {0x42u, 0x43u};
    uint8_t mutable_data[] = {'S', 'T', 'M'};
    uint8_t too_long[MBRTU_SERVER_ID_MAX_ADDITIONAL_DATA + 1u] = {0u};
    uint8_t too_long_server_id[
        MBRTU_SERVER_ID_MAX_SERVER_ID_LENGTH + 1u] = {0u};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    mbrtu_server_id_clear();
    CHECK(mbrtu_server_id_is_configured() == 0);
    CHECK(mbrtu_server_id_configure(NULL, 1u,
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    NULL, 0u) ==
          MBRTU_SERVER_ID_ERROR_ARGUMENT);
    CHECK(mbrtu_server_id_configure(mutable_server_id, 0u,
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    NULL, 0u) ==
          MBRTU_SERVER_ID_ERROR_LENGTH);
    CHECK(mbrtu_server_id_configure(too_long_server_id,
                                    sizeof(too_long_server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    NULL, 0u) ==
          MBRTU_SERVER_ID_ERROR_LENGTH);
    CHECK(mbrtu_server_id_configure(mutable_server_id,
                                    sizeof(mutable_server_id),
                                    0x01u, NULL, 0u) ==
          MBRTU_SERVER_ID_ERROR_STATUS);
    CHECK(mbrtu_server_id_configure(mutable_server_id,
                                    sizeof(mutable_server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    NULL, 1u) ==
          MBRTU_SERVER_ID_ERROR_ARGUMENT);
    CHECK(mbrtu_server_id_configure(mutable_server_id,
                                    sizeof(mutable_server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    too_long, sizeof(too_long)) ==
          MBRTU_SERVER_ID_ERROR_CAPACITY);
    CHECK(mbrtu_server_id_configure(
              mutable_server_id, sizeof(mutable_server_id),
              MBRTU_SERVER_ID_RUN_STATUS_ON, too_long,
              MBRTU_SERVER_ID_MAX_ADDITIONAL_DATA) ==
          MBRTU_SERVER_ID_ERROR_CAPACITY);

    CHECK(mbrtu_server_id_configure(mutable_server_id,
                                    sizeof(mutable_server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    mutable_data,
                                    sizeof(mutable_data)) ==
          MBRTU_SERVER_ID_OK);
    CHECK(mbrtu_server_id_is_configured() != 0);
    mutable_server_id[0] = 0u;
    mutable_data[0] = 'X';

    request_length = make_request(1u, NULL, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response_length == 11u);
    CHECK(response[1] == MBRTU_SERVER_ID_FUNCTION_CODE);
    CHECK(response[2] == 6u);
    CHECK(response[3] == 0x42u);
    CHECK(response[4] == 0x43u);
    CHECK(response[4] == 0x43u);
    CHECK(response[5] == MBRTU_SERVER_ID_RUN_STATUS_ON);
    CHECK(memcmp(&response[6], "STM", 3u) == 0);
    CHECK(mb_crc16(response, response_length) == 0u);

    CHECK(mbrtu_server_id_configure(mutable_server_id,
                                    sizeof(mutable_server_id),
                                    0x01u, NULL, 0u) ==
          MBRTU_SERVER_ID_ERROR_STATUS);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response[3] == 0x42u);

    mbrtu_server_id_clear();
    CHECK(mbrtu_server_id_is_configured() == 0);
    return EXIT_SUCCESS;
}

static int test_rtu_and_tcp_paths(void)
{
    const uint8_t server_id[] = {0x7Eu, 0x33u};
    const uint8_t additional_data[] = {0xAAu, 0x55u};
    const uint8_t extra[] = {0x00u};
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t tcp_request[] = {
        0x12u, 0x34u, 0x00u, 0x00u, 0x00u, 0x02u, 0x01u, 0x11u
    };
    uint8_t tcp_response[MODBUS_TCP_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    CHECK(mbrtu_server_id_configure(server_id, sizeof(server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_OFF,
                                    additional_data,
                                    sizeof(additional_data)) == 0);

    request_length = make_request(1u, NULL, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, 3u, &response_length) ==
          MBRTU_ERROR_RESPONSE_CAPACITY);
    CHECK(response_length == 0u);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response_length == 10u);
    CHECK(memcmp(&response[1],
                 (const uint8_t[]){0x11u, 0x05u, 0x7Eu, 0x33u,
                                   0x00u, 0xAAu, 0x55u},
                 7u) == 0);

    request_length = make_request(1u, extra, sizeof(extra), request);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response_length == 5u);
    CHECK(response[1] == 0x91u && response[2] == 0x03u);

    request_length = make_request(0u, NULL, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_NO_RESPONSE);
    CHECK(response_length == 0u);

    request_length = make_request(2u, NULL, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_NO_RESPONSE);
    request[request_length - 1u] ^= 1u;
    CHECK(mbrtu_process_adu(2u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_NO_RESPONSE);

    request_length = make_request(1u, NULL, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, 9u, &response_length) ==
          MBRTU_ERROR_RESPONSE_CAPACITY);

    mbrtu_server_id_clear();
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response[1] == 0x91u && response[2] == 0x04u);

    CHECK(mbtcp_process_adu(tcp_request, sizeof(tcp_request),
                            tcp_response, sizeof(tcp_response),
                            &response_length) == 0);
    CHECK(response_length == 9u);
    CHECK(tcp_response[7] == 0x91u && tcp_response[8] == 0x01u);
    return EXIT_SUCCESS;
}

static int test_maximum_response(void)
{
    uint8_t server_id[MBRTU_SERVER_ID_MAX_SERVER_ID_LENGTH];
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    for (size_t i = 0u; i < sizeof(server_id); ++i) {
        server_id[i] = (uint8_t)i;
    }
    CHECK(mbrtu_server_id_configure(
              server_id, sizeof(server_id),
              MBRTU_SERVER_ID_RUN_STATUS_ON, NULL, 0u) == 0);
    request_length = make_request(1u, NULL, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length,
                            response, sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response_length == MODBUS_RTU_ADU_MAX_SIZE);
    CHECK(response[2] == MBRTU_SERVER_ID_MAX_BYTE_COUNT);
    CHECK(memcmp(&response[3], server_id, sizeof(server_id)) == 0);
    CHECK(response[3u + sizeof(server_id)] ==
          MBRTU_SERVER_ID_RUN_STATUS_ON);
    CHECK(mb_crc16(response, response_length) == 0u);
    return EXIT_SUCCESS;
}

static int test_master_builder_and_response(void)
{
    const uint8_t server_id[] = {0x31u, 0x32u};
    const uint8_t additional_data[] = {'I', 'D'};
    mbrtum_request_t request;
    mbrtum_response_t response;
    mbrtum_server_id_response_t server_id_response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(mbrtum_build_report_server_id_request(
              1u, sizeof(server_id), &request, request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    CHECK(request_length == 4u);
    CHECK(request.function == MBRTUM_FC_REPORT_SERVER_ID);
    CHECK(request.expects_response == 1u);
    CHECK(request.quantity == sizeof(server_id));
    CHECK(request_adu[0] == 1u && request_adu[1] == 0x11u);
    CHECK(mb_crc16(request_adu, request_length) == 0u);
    CHECK(mbrtum_build_report_server_id_request(
              0u, sizeof(server_id), &request, request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(mbrtum_build_report_server_id_request(
              1u, sizeof(server_id), &request, request_adu, 3u,
              &request_length) == MBRTUM_ERROR_CAPACITY);
    CHECK(mbrtum_build_report_server_id_request(
              1u, 0u, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_ERROR_QUANTITY);

    CHECK(mbrtum_build_report_server_id_request(
              1u, sizeof(server_id), &request, request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    response_length = make_response(1u, server_id, sizeof(server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    additional_data,
                                    sizeof(additional_data),
                                    response_adu);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.function == MBRTUM_FC_REPORT_SERVER_ID);
    CHECK(response.data_length == 5u);
    CHECK(mbrtum_get_server_id_response(&request, &response,
                                         &server_id_response) == MBRTUM_OK);
    CHECK(server_id_response.server_id_length == sizeof(server_id));
    CHECK(memcmp(server_id_response.server_id, server_id,
                 sizeof(server_id)) == 0);
    CHECK(server_id_response.run_status == MBRTU_SERVER_ID_RUN_STATUS_ON);
    CHECK(server_id_response.additional_data_length == 2u);
    CHECK(memcmp(server_id_response.additional_data, "ID", 2u) == 0);

    {
        mbrtum_request_t wrong_length_request = request;

        wrong_length_request.quantity = 1u;
        CHECK(mbrtum_process_response(&wrong_length_request,
                                      response_adu, response_length,
                                      &response) ==
              MBRTUM_ERROR_MALFORMED_RESPONSE);
    }

    response_adu[2] = 3u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_RESPONSE_LENGTH);
    response_length = make_response(1u, server_id, sizeof(server_id),
                                    0x01u, NULL, 0u, response_adu);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);
    response_length = make_response(2u, server_id, sizeof(server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    NULL, 0u, response_adu);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) ==
          MBRTUM_ERROR_ADDRESS_MISMATCH);
    response_adu[response_length - 1u] ^= 1u;
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_CRC);
    return EXIT_SUCCESS;
}

static int test_transaction_engine(void)
{
    const uint8_t server_id[] = {0x55u, 0x56u};
    const uint8_t additional_data[] = {0xA5u};
    fake_transport_t transport = {0u, {0u}, 0u};
    mbrtum_transaction_t transaction;
    mbrtum_transaction_config_t config = {100u, 0u, 0u, 1u};
    mbrtum_request_t request;
    mbrtum_server_id_response_t server_id_response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(mbrtum_build_report_server_id_request(
              1u, sizeof(server_id), &request, request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&transaction, &config,
                                  fake_transmit, &transport) ==
          MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 10u) ==
          MBRTUM_TXN_OK);
    CHECK(transport.calls == 1u);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 11u) ==
          MBRTUM_TXN_OK);
    response_length = make_response(1u, server_id, sizeof(server_id),
                                    MBRTU_SERVER_ID_RUN_STATUS_ON,
                                    additional_data,
                                    sizeof(additional_data),
                                    response_adu);
    CHECK(mbrtum_transaction_on_response(&transaction,
                                         response_adu,
                                         response_length,
                                         12u) == MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(mbrtum_get_server_id_response(&transaction.request,
                                         &transaction.response,
                                         &server_id_response) == MBRTUM_OK);
    CHECK(memcmp(server_id_response.server_id, server_id,
                 sizeof(server_id)) == 0);

    request.quantity = 0u;
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 20u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_configuration() == EXIT_SUCCESS);
    CHECK(test_rtu_and_tcp_paths() == EXIT_SUCCESS);
    CHECK(test_maximum_response() == EXIT_SUCCESS);
    CHECK(test_master_builder_and_response() == EXIT_SUCCESS);
    CHECK(test_transaction_engine() == EXIT_SUCCESS);

    puts("Modbus FC11 Report Server ID tests: PASS");
    return EXIT_SUCCESS;
}
