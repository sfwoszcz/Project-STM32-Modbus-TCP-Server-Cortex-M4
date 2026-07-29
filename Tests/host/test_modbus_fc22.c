#include "modbus.h"
#include "modbus_crc16.h"
#include "modbus_pdu.h"
#include "modbus_protocol.h"
#include "modbus_rtu.h"
#include "modbus_rtu_master.h"
#include "modbus_rtu_master_transaction.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

static uint32_t write_hook_calls;
static uint16_t write_hook_address;
static uint16_t write_hook_value;

void mb_on_write_hreg(uint16_t address, uint16_t value)
{
    ++write_hook_calls;
    write_hook_address = address;
    write_hook_value = value;
}

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

static size_t make_rtu_adu(uint8_t address,
                           const uint8_t *pdu,
                           size_t pdu_length,
                           uint8_t *adu)
{
    adu[0] = address;
    memcpy(&adu[1], pdu, pdu_length);
    return append_crc(adu, 1u + pdu_length);
}

static size_t make_tcp_adu(const uint8_t *pdu,
                           size_t pdu_length,
                           uint8_t *adu)
{
    uint16_t length = (uint16_t)(1u + pdu_length);

    adu[0] = 0x12u;
    adu[1] = 0x34u;
    adu[2] = 0u;
    adu[3] = 0u;
    adu[4] = (uint8_t)(length >> 8u);
    adu[5] = (uint8_t)length;
    adu[6] = 1u;
    memcpy(&adu[7], pdu, pdu_length);
    return MBTCP_MBAP_HEADER_SIZE + pdu_length;
}

static int process_pdu_expect(const uint8_t *request,
                              size_t request_length,
                              const uint8_t *expected,
                              size_t expected_length)
{
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    CHECK(mb_process_pdu(request,
                         request_length,
                         response,
                         sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == expected_length);
    CHECK(memcmp(response, expected, expected_length) == 0);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_mask_semantics(void)
{
    const uint8_t request[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x00u, 0x05u,
        0x00u, 0xFFu,
        0xFFu, 0x00u
    };
    const uint8_t keep_request[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x00u, 0x05u,
        0xFFu, 0xFFu,
        0x00u, 0x00u
    };
    const uint8_t replace_request[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x00u, 0x05u,
        0x00u, 0x00u,
        0xA5u, 0x5Au
    };

    mb_init();
    mb_set_hreg(5u, 0x1234u);
    write_hook_calls = 0u;

    CHECK(process_pdu_expect(request, sizeof(request), request,
                             sizeof(request)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(5u) == 0xFF34u);
    CHECK(write_hook_calls == 1u);
    CHECK(write_hook_address == 5u);
    CHECK(write_hook_value == 0xFF34u);

    CHECK(process_pdu_expect(keep_request, sizeof(keep_request), keep_request,
                             sizeof(keep_request)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(5u) == 0xFF34u);

    CHECK(process_pdu_expect(replace_request, sizeof(replace_request),
                             replace_request, sizeof(replace_request)) ==
          EXIT_SUCCESS);
    CHECK(mb_get_hreg(5u) == 0xA55Au);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_validation_and_capacity(void)
{
    const uint8_t short_request[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x00u, 0x05u,
        0xFFu, 0x00u,
        0x12u
    };
    const uint8_t long_request[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x00u, 0x05u,
        0xFFu, 0x00u,
        0x12u, 0x34u,
        0x00u
    };
    const uint8_t bad_address[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x01u, 0x00u,
        0xFFu, 0x00u,
        0x12u, 0x34u
    };
    const uint8_t value_error[] = {0x96u, 0x03u};
    const uint8_t address_error[] = {0x96u, 0x02u};
    const uint8_t capacity_request[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x00u, 0x05u,
        0x00u, 0x00u,
        0xBEu, 0xEFu
    };
    uint8_t response[2];
    size_t response_length = 0u;

    mb_init();
    CHECK(process_pdu_expect(short_request, sizeof(short_request),
                             value_error, sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(long_request, sizeof(long_request),
                             value_error, sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(bad_address, sizeof(bad_address),
                             address_error, sizeof(address_error)) ==
          EXIT_SUCCESS);

    mb_set_hreg(5u, 0x1111u);
    write_hook_calls = 0u;
    CHECK(mb_process_pdu(capacity_request,
                         sizeof(capacity_request),
                         response,
                         sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == 2u);
    CHECK(response[0] == 0x96u && response[1] == 0x04u);
    CHECK(mb_get_hreg(5u) == 0x1111u);
    CHECK(write_hook_calls == 0u);
    return EXIT_SUCCESS;
}

static int test_tcp_rtu_and_broadcast_paths(void)
{
    const uint8_t pdu[] = {
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x00u, 0x07u,
        0x0Fu, 0xF0u,
        0x50u, 0x05u
    };
    uint8_t request[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    mb_init();
    mb_set_hreg(7u, 0xABCDu);
    request_length = make_tcp_adu(pdu, sizeof(pdu), request);
    CHECK(mbtcp_process_adu(request,
                            request_length,
                            response,
                            sizeof(response),
                            &response_length) == 0);
    CHECK(response_length == MBTCP_MBAP_HEADER_SIZE + sizeof(pdu));
    CHECK(memcmp(&response[7], pdu, sizeof(pdu)) == 0);
    CHECK(mb_get_hreg(7u) == 0x5BC5u);

    mb_init();
    mb_set_hreg(7u, 0xABCDu);
    request_length = make_rtu_adu(1u, pdu, sizeof(pdu), request);
    response_length = 0u;
    CHECK(mbrtu_process_adu(1u,
                            request,
                            request_length,
                            response,
                            sizeof(response),
                            &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response_length == 1u + sizeof(pdu) + MODBUS_RTU_CRC_SIZE);
    CHECK(memcmp(&response[1], pdu, sizeof(pdu)) == 0);
    CHECK(mb_get_hreg(7u) == 0x5BC5u);

    mb_init();
    mb_set_hreg(7u, 0xABCDu);
    request_length = make_rtu_adu(MODBUS_RTU_BROADCAST_ADDRESS,
                                  pdu,
                                  sizeof(pdu),
                                  request);
    response_length = 123u;
    CHECK(mbrtu_process_adu(1u,
                            request,
                            request_length,
                            response,
                            sizeof(response),
                            &response_length) == MBRTU_NO_RESPONSE);
    CHECK(response_length == 0u);
    CHECK(mb_get_hreg(7u) == 0x5BC5u);

    mb_set_hreg(7u, 0x2468u);
    request_length = make_rtu_adu(MODBUS_RTU_BROADCAST_ADDRESS,
                                  pdu,
                                  sizeof(pdu) - 1u,
                                  request);
    response_length = 123u;
    CHECK(mbrtu_process_adu(1u,
                            request,
                            request_length,
                            response,
                            sizeof(response),
                            &response_length) == MBRTU_NO_RESPONSE);
    CHECK(response_length == 0u);
    CHECK(mb_get_hreg(7u) == 0x2468u);
    return EXIT_SUCCESS;
}

static int test_master_builder(void)
{
    const uint8_t expected_without_crc[] = {
        1u,
        MBRTUM_FC_MASK_WRITE_REGISTER,
        0x12u, 0x34u,
        0xF0u, 0x0Fu,
        0x0Fu, 0xF0u
    };
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 99u;

    CHECK(mbrtum_build_mask_write_register_request(
              1u, 0x1234u, 0xF00Fu, 0x0FF0u, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 10u);
    CHECK(memcmp(adu, expected_without_crc,
                 sizeof(expected_without_crc)) == 0);
    CHECK(mb_crc16(adu, adu_length) == 0u);
    CHECK(request.slave_address == 1u);
    CHECK(request.function == MBRTUM_FC_MASK_WRITE_REGISTER);
    CHECK(request.start_address == 0x1234u);
    CHECK(request.quantity == 0xF00Fu);
    CHECK(request.value == 0x0FF0u);
    CHECK(request.expects_response == 1u);
    CHECK(request.write_start_address == 0u);
    CHECK(request.write_quantity == 0u);

    CHECK(mbrtum_build_mask_write_register_request(
              0u, 1u, 2u, 3u, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(request.expects_response == 0u);
    CHECK(adu[0] == 0u && adu[1] == MBRTUM_FC_MASK_WRITE_REGISTER);

    adu_length = 99u;
    memset(&request, 0xA5, sizeof(request));
    CHECK(mbrtum_build_mask_write_register_request(
              248u, 1u, 2u, 3u, &request,
              adu, sizeof(adu), &adu_length) ==
          MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(adu_length == 0u);
    CHECK(request.function == 0u);

    CHECK(mbrtum_build_mask_write_register_request(
              1u, 1u, 2u, 3u, &request,
              adu, 9u, &adu_length) == MBRTUM_ERROR_CAPACITY);
    CHECK(adu_length == 0u);
    return EXIT_SUCCESS;
}

static int test_master_response_validation(void)
{
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(mbrtum_build_mask_write_register_request(
              1u, 0x1234u, 0xF00Fu, 0x0FF0u, &request,
              request_adu, sizeof(request_adu), &request_length) == MBRTUM_OK);
    memcpy(response_adu, request_adu, request_length);
    response_length = request_length;
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.function == MBRTUM_FC_MASK_WRITE_REGISTER);
    CHECK(response.data == NULL && response.data_length == 0u);

    response_adu[6] ^= 1u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) ==
          MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH);

    memcpy(response_adu, request_adu, request_length);
    response_adu[4] ^= 1u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) ==
          MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH);

    memcpy(response_adu, request_adu, 7u);
    response_length = append_crc(response_adu, 7u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) == MBRTUM_ERROR_RESPONSE_LENGTH);

    memcpy(response_adu, request_adu, request_length);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  request_length - 1u,
                                  &response) == MBRTUM_ERROR_CRC);

    {
        const uint8_t exception_pdu[] = {0x96u, 0x02u};

        response_length = make_rtu_adu(1u, exception_pdu,
                                       sizeof(exception_pdu), response_adu);
        CHECK(mbrtum_process_response(&request,
                                      response_adu,
                                      response_length,
                                      &response) == MBRTUM_EXCEPTION_RESPONSE);
        CHECK(response.exception_code == 0x02u);
    }

    CHECK(mbrtum_build_mask_write_register_request(
              0u, 1u, 2u, 3u, &request,
              request_adu, sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) ==
          MBRTUM_ERROR_RESPONSE_NOT_EXPECTED);
    return EXIT_SUCCESS;
}

typedef struct {
    uint32_t calls;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length;
} fake_transport_t;

static int fake_transmit(void *context, const uint8_t *adu, size_t adu_length)
{
    fake_transport_t *transport = (fake_transport_t *)context;

    ++transport->calls;
    memcpy(transport->adu, adu, adu_length);
    transport->adu_length = adu_length;
    return MBRTUM_TXN_TRANSMIT_ACCEPTED;
}

static int test_transaction_engine(void)
{
    const mbrtum_transaction_config_t config = {
        .response_timeout_ticks = 20u,
        .retry_delay_ticks = 0u,
        .max_retries = 0u,
        .retry_transport_errors = 0u
    };
    fake_transport_t transport = {0};
    mbrtum_transaction_t transaction;
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;

    CHECK(mbrtum_build_mask_write_register_request(
              1u, 5u, 0x00FFu, 0xFF00u, &request,
              request_adu, sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&transaction, &config,
                                  fake_transmit, &transport) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 100u) ==
          MBRTUM_TXN_OK);
    CHECK(transport.calls == 1u);
    CHECK(transaction.state == MBRTUM_TXN_STATE_TRANSMITTING);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 101u) ==
          MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_WAITING_RESPONSE);

    memcpy(response_adu, request_adu, request_length);
    CHECK(mbrtum_transaction_on_response(&transaction,
                                         response_adu,
                                         request_length,
                                         102u) == MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(transaction.protocol_result == MBRTUM_OK);

    request.quantity ^= 1u;
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 200u) ==
          MBRTUM_TXN_ERROR_REQUEST);

    CHECK(mbrtum_build_mask_write_register_request(
              0u, 5u, 0x00FFu, 0xFF00u, &request,
              request_adu, sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 300u) ==
          MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 301u) ==
          MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_shared_pdu_mask_semantics() == EXIT_SUCCESS);
    CHECK(test_shared_pdu_validation_and_capacity() == EXIT_SUCCESS);
    CHECK(test_tcp_rtu_and_broadcast_paths() == EXIT_SUCCESS);
    CHECK(test_master_builder() == EXIT_SUCCESS);
    CHECK(test_master_response_validation() == EXIT_SUCCESS);
    CHECK(test_transaction_engine() == EXIT_SUCCESS);

    puts("Modbus FC22 Mask Write Register tests: PASS");
    return EXIT_SUCCESS;
}
