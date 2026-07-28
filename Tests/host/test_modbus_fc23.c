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

static void write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8u);
    p[1] = (uint8_t)value;
}

static size_t append_crc(uint8_t *adu, size_t length_without_crc)
{
    uint16_t crc = mb_crc16(adu, length_without_crc);

    adu[length_without_crc] = (uint8_t)crc;
    adu[length_without_crc + 1u] = (uint8_t)(crc >> 8u);
    return length_without_crc + MODBUS_RTU_CRC_SIZE;
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

static int test_shared_pdu_write_before_read(void)
{
    const uint8_t request[] = {
        0x17u,
        0x00u, 0x03u,
        0x00u, 0x06u,
        0x00u, 0x04u,
        0x00u, 0x03u,
        0x06u,
        0x00u, 0xFFu,
        0x00u, 0xFFu,
        0x00u, 0xFFu
    };
    const uint8_t expected[] = {
        0x17u, 0x0Cu,
        0x10u, 0x03u,
        0x00u, 0xFFu,
        0x00u, 0xFFu,
        0x00u, 0xFFu,
        0x10u, 0x07u,
        0x10u, 0x08u
    };

    mb_init();
    for (uint16_t i = 0u; i < 16u; ++i) {
        mb_set_hreg(i, (uint16_t)(0x1000u + i));
    }

    CHECK(process_pdu_expect(request,
                             sizeof(request),
                             expected,
                             sizeof(expected)) == EXIT_SUCCESS);
    CHECK(mb_get_hreg(4u) == 0x00FFu);
    CHECK(mb_get_hreg(5u) == 0x00FFu);
    CHECK(mb_get_hreg(6u) == 0x00FFu);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_validation_and_transactionality(void)
{
    const uint8_t short_request[] = {
        0x17u, 0x00u, 0x00u, 0x00u, 0x01u,
        0x00u, 0x00u, 0x00u, 0x01u
    };
    const uint8_t bad_read_quantity[] = {
        0x17u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x01u, 0x02u, 0x12u, 0x34u
    };
    const uint8_t bad_write_quantity[] = {
        0x17u, 0x00u, 0x00u, 0x00u, 0x01u,
        0x00u, 0x00u, 0x00u, 0x7Au, 0x00u
    };
    const uint8_t bad_byte_count[] = {
        0x17u, 0x00u, 0x00u, 0x00u, 0x01u,
        0x00u, 0x01u, 0x00u, 0x02u, 0x02u, 0x12u, 0x34u
    };
    const uint8_t bad_read_address[] = {
        0x17u, 0x00u, 0xFFu, 0x00u, 0x02u,
        0x00u, 0x01u, 0x00u, 0x01u, 0x02u, 0x12u, 0x34u
    };
    const uint8_t bad_write_address[] = {
        0x17u, 0x00u, 0x01u, 0x00u, 0x01u,
        0x00u, 0xFFu, 0x00u, 0x02u, 0x04u,
        0x12u, 0x34u, 0x56u, 0x78u
    };
    const uint8_t capacity_request[] = {
        0x17u, 0x00u, 0x00u, 0x00u, 0x02u,
        0x00u, 0x05u, 0x00u, 0x01u, 0x02u, 0xCAu, 0xFEu
    };
    const uint8_t expected_value[] = {0x97u, 0x03u};
    const uint8_t expected_address[] = {0x97u, 0x02u};
    uint8_t response[2];
    size_t response_length = 0u;

    mb_init();
    CHECK(process_pdu_expect(short_request, sizeof(short_request),
                             expected_value, sizeof(expected_value)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(bad_read_quantity, sizeof(bad_read_quantity),
                             expected_value, sizeof(expected_value)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(bad_write_quantity, sizeof(bad_write_quantity),
                             expected_value, sizeof(expected_value)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(bad_byte_count, sizeof(bad_byte_count),
                             expected_value, sizeof(expected_value)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(bad_read_address, sizeof(bad_read_address),
                             expected_address, sizeof(expected_address)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(bad_write_address, sizeof(bad_write_address),
                             expected_address, sizeof(expected_address)) == EXIT_SUCCESS);

    mb_set_hreg(5u, 0x1111u);
    CHECK(mb_process_pdu(capacity_request,
                         sizeof(capacity_request),
                         response,
                         sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == 2u);
    CHECK(response[0] == 0x97u && response[1] == 0x04u);
    CHECK(mb_get_hreg(5u) == 0x1111u);
    return EXIT_SUCCESS;
}

static int test_maximum_pdu(void)
{
    uint8_t request[MODBUS_PDU_MAX_SIZE];
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t request_length = 10u + (121u * 2u);
    size_t response_length = 0u;

    mb_init();
    request[0] = 0x17u;
    write_be16(&request[1], 0u);
    write_be16(&request[3], 125u);
    write_be16(&request[5], 125u);
    write_be16(&request[7], 121u);
    request[9] = 242u;
    for (uint16_t i = 0u; i < 121u; ++i) {
        write_be16(&request[10u + ((size_t)i * 2u)],
                   (uint16_t)(0x2000u + i));
    }

    CHECK(request_length == 252u);
    CHECK(mb_process_pdu(request,
                         request_length,
                         response,
                         sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == 252u);
    CHECK(response[0] == 0x17u && response[1] == 250u);
    CHECK(mb_get_hreg(125u) == 0x2000u);
    CHECK(mb_get_hreg(245u) == 0x2078u);
    return EXIT_SUCCESS;
}

static int test_tcp_and_rtu_paths(void)
{
    const uint8_t request_pdu[] = {
        0x17u,
        0x00u, 0x02u,
        0x00u, 0x03u,
        0x00u, 0x03u,
        0x00u, 0x02u,
        0x04u,
        0xAAu, 0xAAu,
        0xBBu, 0xBBu
    };
    const uint8_t expected_pdu[] = {
        0x17u, 0x06u,
        0x10u, 0x02u,
        0xAAu, 0xAAu,
        0xBBu, 0xBBu
    };
    uint8_t request_adu[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_TCP_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    mb_init();
    mb_set_hreg(2u, 0x1002u);
    mb_set_hreg(3u, 0x1003u);
    mb_set_hreg(4u, 0x1004u);
    request_length = make_tcp_adu(request_pdu, sizeof(request_pdu), request_adu);
    CHECK(mbtcp_process_adu(request_adu,
                            request_length,
                            response_adu,
                            sizeof(response_adu),
                            &response_length) == 0);
    CHECK(response_length == MBTCP_MBAP_HEADER_SIZE + sizeof(expected_pdu));
    CHECK(memcmp(&response_adu[7], expected_pdu, sizeof(expected_pdu)) == 0);

    mb_init();
    mb_set_hreg(2u, 0x1002u);
    mb_set_hreg(3u, 0x1003u);
    mb_set_hreg(4u, 0x1004u);
    request_length = make_rtu_adu(1u, request_pdu, sizeof(request_pdu), request_adu);
    response_length = 0u;
    CHECK(mbrtu_process_adu(1u,
                            request_adu,
                            request_length,
                            response_adu,
                            sizeof(response_adu),
                            &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response_length == 1u + sizeof(expected_pdu) + MODBUS_RTU_CRC_SIZE);
    CHECK(response_adu[0] == 1u);
    CHECK(memcmp(&response_adu[1], expected_pdu, sizeof(expected_pdu)) == 0);
    CHECK(mb_crc16(response_adu, response_length) == 0u);

    mb_init();
    mb_set_hreg(3u, 0x3333u);
    request_length = make_rtu_adu(0u, request_pdu, sizeof(request_pdu), request_adu);
    response_length = 99u;
    CHECK(mbrtu_process_adu(1u,
                            request_adu,
                            request_length,
                            response_adu,
                            sizeof(response_adu),
                            &response_length) == MBRTU_NO_RESPONSE);
    CHECK(response_length == 0u);
    CHECK(mb_get_hreg(3u) == 0x3333u);
    return EXIT_SUCCESS;
}

static int test_master_builder(void)
{
    const uint16_t values[] = {0x1111u, 0x2222u, 0x3333u};
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 3u, 6u, 14u, 3u, values, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 19u);
    CHECK(adu[0] == 1u && adu[1] == 0x17u);
    CHECK(adu[2] == 0u && adu[3] == 3u);
    CHECK(adu[4] == 0u && adu[5] == 6u);
    CHECK(adu[6] == 0u && adu[7] == 14u);
    CHECK(adu[8] == 0u && adu[9] == 3u);
    CHECK(adu[10] == 6u);
    CHECK(memcmp(&adu[11], (const uint8_t[]){
        0x11u, 0x11u, 0x22u, 0x22u, 0x33u, 0x33u
    }, 6u) == 0);
    CHECK(mb_crc16(adu, adu_length) == 0u);
    CHECK(request.slave_address == 1u);
    CHECK(request.function == MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS);
    CHECK(request.start_address == 3u && request.quantity == 6u);
    CHECK(request.write_start_address == 14u && request.write_quantity == 3u);
    CHECK(request.value == 0u && request.expects_response == 1u);

    CHECK(mbrtum_build_read_write_multiple_registers_request(
              0u, 0u, 1u, 0u, 1u, values, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(adu_length == 0u);
    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 0u, 126u, 0u, 1u, values, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 0u, 1u, 0u, 122u, values, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 0xFFFFu, 2u, 0u, 1u, values, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 0u, 1u, 0xFFFFu, 2u, values, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 0u, 1u, 0u, 1u, values, &request,
              adu, 14u, &adu_length) == MBRTUM_ERROR_CAPACITY);
    return EXIT_SUCCESS;
}

static int test_master_maximum_request(void)
{
    uint16_t values[121];
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    for (uint16_t i = 0u; i < 121u; ++i) {
        values[i] = (uint16_t)(0x3000u + i);
    }
    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 0u, 125u, 125u, 121u, values, &request,
              adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 255u);
    CHECK(adu[10] == 242u);
    CHECK(mb_crc16(adu, adu_length) == 0u);
    return EXIT_SUCCESS;
}

static int test_master_response_and_decoder(void)
{
    const uint16_t values[] = {0xAAAAu, 0xBBBBu};
    const uint8_t response_pdu[] = {
        0x17u, 0x06u,
        0x1234u >> 8u, (uint8_t)0x1234u,
        0xABCDu >> 8u, (uint8_t)0xABCDu,
        0x0001u >> 8u, (uint8_t)0x0001u
    };
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;
    uint16_t value = 0u;

    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 2u, 3u, 5u, 2u, values, &request,
              request_adu, sizeof(request_adu), &request_length) == MBRTUM_OK);
    response_length = make_rtu_adu(1u, response_pdu,
                                   sizeof(response_pdu), response_adu);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.function == MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS);
    CHECK(response.data_length == 6u);
    CHECK(mbrtum_get_register(&request, &response, 0u, &value) == MBRTUM_OK);
    CHECK(value == 0x1234u);
    CHECK(mbrtum_get_register(&request, &response, 1u, &value) == MBRTUM_OK);
    CHECK(value == 0xABCDu);
    CHECK(mbrtum_get_register(&request, &response, 2u, &value) == MBRTUM_OK);
    CHECK(value == 0x0001u);
    CHECK(mbrtum_get_register(&request, &response, 3u, &value) ==
          MBRTUM_ERROR_INDEX);

    response_adu[2] = 4u;
    response_adu[response_length - 2u] = 0u;
    response_adu[response_length - 1u] = 0u;
    response_length = append_crc(response_adu, response_length - 2u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) == MBRTUM_ERROR_MALFORMED_RESPONSE);
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
    const uint16_t values[] = {0xAAAAu, 0xBBBBu};
    const uint8_t response_pdu[] = {
        0x17u, 0x04u, 0xCAu, 0xFEu, 0xBEu, 0xEFu
    };
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
    size_t response_length;
    uint16_t value = 0u;

    CHECK(mbrtum_build_read_write_multiple_registers_request(
              1u, 2u, 2u, 5u, 2u, values, &request,
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

    response_length = make_rtu_adu(1u, response_pdu,
                                   sizeof(response_pdu), response_adu);
    CHECK(mbrtum_transaction_on_response(&transaction,
                                         response_adu,
                                         response_length,
                                         102u) == MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(transaction.protocol_result == MBRTUM_OK);
    CHECK(mbrtum_get_register(&transaction.request,
                              &transaction.response,
                              1u,
                              &value) == MBRTUM_OK);
    CHECK(value == 0xBEEFu);

    request.write_quantity = 3u;
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 200u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_shared_pdu_write_before_read() == EXIT_SUCCESS);
    CHECK(test_shared_pdu_validation_and_transactionality() == EXIT_SUCCESS);
    CHECK(test_maximum_pdu() == EXIT_SUCCESS);
    CHECK(test_tcp_and_rtu_paths() == EXIT_SUCCESS);
    CHECK(test_master_builder() == EXIT_SUCCESS);
    CHECK(test_master_maximum_request() == EXIT_SUCCESS);
    CHECK(test_master_response_and_decoder() == EXIT_SUCCESS);
    CHECK(test_transaction_engine() == EXIT_SUCCESS);

    puts("Modbus FC23 tests: PASS");
    return EXIT_SUCCESS;
}
