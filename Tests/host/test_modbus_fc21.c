#include "modbus.h"
#include "modbus_crc16.h"
#include "modbus_file_record.h"
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

static uint16_t file3[32];
static uint16_t file4[MB_FILE_RECORD_MAX_RECORDS_PER_FILE];

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

static int configure_files(void)
{
    const mb_file_record_file_t files[] = {
        {3u, file3, sizeof(file3) / sizeof(file3[0])},
        {4u, file4, sizeof(file4) / sizeof(file4[0])}
    };

    memset(file3, 0, sizeof(file3));
    memset(file4, 0, sizeof(file4));
    mb_init();
    return mb_file_record_configure(files,
                                    sizeof(files) / sizeof(files[0]));
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

static int test_shared_pdu_example_and_atomic_validation(void)
{
    const uint8_t request[] = {
        0x15u, 0x0Du,
        0x06u, 0x00u, 0x04u, 0x00u, 0x07u, 0x00u, 0x03u,
        0x06u, 0xAFu, 0x04u, 0xBEu, 0x10u, 0x0Du
    };
    const uint8_t value_error[] = {0x95u, 0x03u};
    const uint8_t address_error[] = {0x95u, 0x02u};
    uint8_t malformed[40];
    uint8_t response[2];
    size_t response_length = 0u;

    CHECK(configure_files() == MB_FILE_RECORD_OK);
    CHECK(process_pdu_expect(request, sizeof(request), request,
                             sizeof(request)) == EXIT_SUCCESS);
    CHECK(file4[7] == 0x06AFu);
    CHECK(file4[8] == 0x04BEu);
    CHECK(file4[9] == 0x100Du);

    memcpy(malformed, request, sizeof(request));
    malformed[1] = 8u;
    CHECK(process_pdu_expect(malformed, sizeof(request), value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(request));
    malformed[1] = 14u;
    CHECK(process_pdu_expect(malformed, sizeof(request), value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(request, sizeof(request) - 1u, value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);

    memcpy(malformed, request, sizeof(request));
    malformed[2] = 5u;
    CHECK(process_pdu_expect(malformed, sizeof(request), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(request));
    malformed[3] = 0u;
    malformed[4] = 0u;
    CHECK(process_pdu_expect(malformed, sizeof(request), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(request));
    malformed[4] = 5u;
    CHECK(process_pdu_expect(malformed, sizeof(request), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(request));
    malformed[5] = 0x27u;
    malformed[6] = 0x10u;
    CHECK(process_pdu_expect(malformed, sizeof(request), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(request));
    malformed[7] = 0u;
    malformed[8] = 0u;
    CHECK(process_pdu_expect(malformed, sizeof(request), value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(request));
    malformed[7] = 0u;
    malformed[8] = 4u;
    CHECK(process_pdu_expect(malformed, sizeof(request), value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);

    file4[7] = 0xAAAAu;
    file4[8] = 0xBBBBu;
    {
        const uint8_t atomic_request[] = {
            0x15u, 0x12u,
            0x06u, 0x00u, 0x04u, 0x00u, 0x07u, 0x00u, 0x01u,
            0x12u, 0x34u,
            0x06u, 0x00u, 0x05u, 0x00u, 0x00u, 0x00u, 0x01u,
            0x56u, 0x78u
        };
        CHECK(process_pdu_expect(atomic_request, sizeof(atomic_request),
                                 address_error, sizeof(address_error)) ==
              EXIT_SUCCESS);
        CHECK(file4[7] == 0xAAAAu);
        CHECK(file4[8] == 0xBBBBu);
    }

    CHECK(mb_process_pdu(request, sizeof(request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response_length == 2u);
    CHECK(response[0] == 0x95u && response[1] == 0x04u);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_multiple_overlap_and_maximum(void)
{
    const uint8_t request[] = {
        0x15u, 0x16u,
        0x06u, 0x00u, 0x04u, 0x00u, 0x01u, 0x00u, 0x02u,
        0x11u, 0x11u, 0x22u, 0x22u,
        0x06u, 0x00u, 0x04u, 0x00u, 0x02u, 0x00u, 0x02u,
        0x33u, 0x33u, 0x44u, 0x44u
    };
    uint8_t max_request[MODBUS_PDU_MAX_SIZE];
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    CHECK(configure_files() == MB_FILE_RECORD_OK);
    CHECK(process_pdu_expect(request, sizeof(request), request,
                             sizeof(request)) == EXIT_SUCCESS);
    CHECK(file4[1] == 0x1111u);
    CHECK(file4[2] == 0x3333u);
    CHECK(file4[3] == 0x4444u);

    memset(max_request, 0, sizeof(max_request));
    max_request[0] = MB_FILE_RECORD_WRITE_FUNCTION_CODE;
    max_request[1] = MB_FILE_RECORD_WRITE_REQUEST_DATA_MAX;
    max_request[2] = MB_FILE_RECORD_REFERENCE_TYPE;
    max_request[4] = 4u;
    max_request[7] = 0u;
    max_request[8] = 122u;
    for (uint16_t i = 0u; i < 122u; ++i) {
        size_t offset = 9u + ((size_t)i * 2u);
        max_request[offset] = (uint8_t)(i >> 8u);
        max_request[offset + 1u] = (uint8_t)i;
    }
    CHECK(mb_process_pdu(max_request, sizeof(max_request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response_length == sizeof(max_request));
    CHECK(memcmp(response, max_request, sizeof(max_request)) == 0);
    CHECK(file4[0] == 0u && file4[121] == 121u);
    return EXIT_SUCCESS;
}

static int test_tcp_and_rtu_paths(void)
{
    const uint8_t pdu[] = {
        0x15u, 0x09u,
        0x06u, 0x00u, 0x04u, 0x00u, 0x05u, 0x00u, 0x01u,
        0xABu, 0xCDu
    };
    uint8_t tcp_request[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t tcp_response[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t rtu_request[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t rtu_response[MODBUS_RTU_ADU_MAX_SIZE];
    size_t tcp_request_length;
    size_t tcp_response_length = 0u;
    size_t rtu_request_length;
    size_t rtu_response_length = 0u;

    CHECK(configure_files() == MB_FILE_RECORD_OK);
    tcp_request_length = make_tcp_adu(pdu, sizeof(pdu), tcp_request);
    CHECK(mbtcp_process_adu(tcp_request, tcp_request_length,
                            tcp_response, sizeof(tcp_response),
                            &tcp_response_length) == 0);
    CHECK(tcp_response_length == tcp_request_length);
    CHECK(memcmp(tcp_response, tcp_request, tcp_request_length) == 0);
    CHECK(file4[5] == 0xABCDu);

    file4[5] = 0u;
    rtu_request_length = make_rtu_adu(1u, pdu, sizeof(pdu), rtu_request);
    CHECK(mbrtu_process_adu(1u, rtu_request, rtu_request_length,
                            rtu_response, sizeof(rtu_response),
                            &rtu_response_length) == MBRTU_RESPONSE_READY);
    CHECK(rtu_response_length == rtu_request_length);
    CHECK(memcmp(rtu_response, rtu_request, rtu_request_length) == 0);
    CHECK(file4[5] == 0xABCDu);

    file4[5] = 0u;
    rtu_request_length = make_rtu_adu(0u, pdu, sizeof(pdu), rtu_request);
    CHECK(mbrtu_process_adu(1u, rtu_request, rtu_request_length,
                            rtu_response, sizeof(rtu_response),
                            &rtu_response_length) == MBRTU_NO_RESPONSE);
    CHECK(rtu_response_length == 0u);
    CHECK(file4[5] == 0xABCDu);

    {
        const uint8_t invalid_pdu[] = {
            0x15u, 0x12u,
            0x06u, 0x00u, 0x04u, 0x00u, 0x05u, 0x00u, 0x01u,
            0x12u, 0x34u,
            0x06u, 0x00u, 0x05u, 0x00u, 0x00u, 0x00u, 0x01u,
            0x56u, 0x78u
        };

        file4[5] = 0xABCDu;
        rtu_request_length = make_rtu_adu(0u, invalid_pdu,
                                           sizeof(invalid_pdu), rtu_request);
        CHECK(mbrtu_process_adu(1u, rtu_request, rtu_request_length,
                                rtu_response, sizeof(rtu_response),
                                &rtu_response_length) == MBRTU_NO_RESPONSE);
        CHECK(rtu_response_length == 0u);
        CHECK(file4[5] == 0xABCDu);
    }
    return EXIT_SUCCESS;
}

static int test_master_builder(void)
{
    const uint16_t values[] = {0x06AFu, 0x04BEu, 0x100Du};
    const mbrtum_write_file_record_request_t subrequest = {
        4u, 7u, 3u, values
    };
    const uint8_t expected_without_crc[] = {
        1u, 0x15u, 0x0Du,
        0x06u, 0x00u, 0x04u, 0x00u, 0x07u, 0x00u, 0x03u,
        0x06u, 0xAFu, 0x04u, 0xBEu, 0x10u, 0x0Du
    };
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    CHECK(mbrtum_build_write_file_record_request(
              1u, &subrequest, 1u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_OK);
    CHECK(adu_length == sizeof(expected_without_crc) + 2u);
    CHECK(memcmp(adu, expected_without_crc, sizeof(expected_without_crc)) == 0);
    CHECK(mb_crc16(adu, adu_length) == 0u);
    CHECK(request.slave_address == 1u);
    CHECK(request.function == MBRTUM_FC_WRITE_FILE_RECORD);
    CHECK(request.start_address == 13u);
    CHECK(request.quantity == 1u);
    CHECK(request.value == 0u);
    CHECK(request.expects_response == 1u);

    CHECK(mbrtum_build_write_file_record_request(
              0u, &subrequest, 1u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_OK);
    CHECK(request.expects_response == 0u);

    CHECK(mbrtum_build_write_file_record_request(
              248u, &subrequest, 1u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(mbrtum_build_write_file_record_request(
              1u, NULL, 1u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_ARGUMENT);
    CHECK(mbrtum_build_write_file_record_request(
              1u, &subrequest, 0u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_write_file_record_request(
              1u, &subrequest, MB_FILE_RECORD_WRITE_MAX_SUBREQUESTS + 1u,
              &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_write_file_record_request(
              1u, &subrequest, 1u, &request, adu,
              sizeof(expected_without_crc) + 1u,
              &adu_length) == MBRTUM_ERROR_CAPACITY);
    {
        mbrtum_write_file_record_request_t invalid = subrequest;
        invalid.file_number = 0u;
        CHECK(mbrtum_build_write_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_VALUE);
        invalid = subrequest;
        invalid.record_data = NULL;
        CHECK(mbrtum_build_write_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_VALUE);
        invalid = subrequest;
        invalid.record_length = 0u;
        CHECK(mbrtum_build_write_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_VALUE);
        invalid = subrequest;
        invalid.record_number = 9999u;
        invalid.record_length = 2u;
        CHECK(mbrtum_build_write_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_VALUE);
    }
    return EXIT_SUCCESS;
}

static int test_master_maximum_request(void)
{
    uint16_t values[122];
    mbrtum_write_file_record_request_t subrequest = {
        1u, 0u, 122u, values
    };
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    for (uint16_t i = 0u; i < 122u; ++i) {
        values[i] = i;
    }
    CHECK(mbrtum_build_write_file_record_request(
              247u, &subrequest, 1u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_OK);
    CHECK(adu_length == MODBUS_RTU_ADU_MAX_SIZE);
    CHECK(adu[2] == MB_FILE_RECORD_WRITE_REQUEST_DATA_MAX);
    CHECK(mb_crc16(adu, adu_length) == 0u);

    subrequest.record_length = 123u;
    CHECK(mbrtum_build_write_file_record_request(
              1u, &subrequest, 1u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_QUANTITY);
    return EXIT_SUCCESS;
}

static int test_master_response_validation(void)
{
    const uint16_t values[] = {0x1111u, 0x2222u};
    const mbrtum_write_file_record_request_t subrequest = {
        4u, 1u, 2u, values
    };
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;

    CHECK(mbrtum_build_write_file_record_request(
              1u, &subrequest, 1u, &request, request_adu,
              sizeof(request_adu), &request_length) == MBRTUM_OK);
    memcpy(response_adu, request_adu, request_length);
    CHECK(mbrtum_process_response(&request, response_adu, request_length,
                                  &response) ==
          MBRTUM_ERROR_REQUEST_DATA_REQUIRED);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, request_length, &response) == MBRTUM_OK);
    CHECK(response.function == MBRTUM_FC_WRITE_FILE_RECORD);
    CHECK(response.data == NULL && response.data_length == 0u);

    response_adu[10] ^= 1u;
    replace_crc(response_adu, request_length);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, request_length, &response) ==
          MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH);

    memcpy(response_adu, request_adu, request_length);
    request_adu[2] = 10u;
    replace_crc(request_adu, request_length);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, request_length, &response) ==
          MBRTUM_ERROR_REQUEST_DATA_REQUIRED);

    {
        const uint8_t exception_pdu[] = {0x95u, 0x02u};
        size_t response_length = make_rtu_adu(1u, exception_pdu,
                                              sizeof(exception_pdu),
                                              response_adu);
        CHECK(mbrtum_build_write_file_record_request(
                  1u, &subrequest, 1u, &request, request_adu,
                  sizeof(request_adu), &request_length) == MBRTUM_OK);
        CHECK(mbrtum_process_response_with_request_adu(
                  &request, request_adu, request_length,
                  response_adu, response_length, &response) ==
              MBRTUM_EXCEPTION_RESPONSE);
        CHECK(response.exception_code == 0x02u);
    }

    CHECK(mbrtum_build_write_file_record_request(
              0u, &subrequest, 1u, &request, request_adu,
              sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              request_adu, request_length, &response) ==
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
    const uint16_t values[] = {0x1234u};
    const mbrtum_write_file_record_request_t subrequest = {
        4u, 0u, 1u, values
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
    size_t request_length = 0u;

    CHECK(mbrtum_build_write_file_record_request(
              1u, &subrequest, 1u, &request, request_adu,
              sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&transaction, &config,
                                  fake_transmit, &transport) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 100u) ==
          MBRTUM_TXN_OK);
    CHECK(transport.calls == 1u);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 101u) ==
          MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_on_response(&transaction,
                                         request_adu,
                                         request_length,
                                         102u) == MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(transaction.protocol_result == MBRTUM_OK);

    request.quantity = 2u;
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 200u) ==
          MBRTUM_TXN_ERROR_REQUEST);

    transport.calls = 0u;
    CHECK(mbrtum_build_write_file_record_request(
              0u, &subrequest, 1u, &request, request_adu,
              sizeof(request_adu), &request_length) == MBRTUM_OK);
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
    CHECK(test_shared_pdu_example_and_atomic_validation() == EXIT_SUCCESS);
    CHECK(test_shared_pdu_multiple_overlap_and_maximum() == EXIT_SUCCESS);
    CHECK(test_tcp_and_rtu_paths() == EXIT_SUCCESS);
    CHECK(test_master_builder() == EXIT_SUCCESS);
    CHECK(test_master_maximum_request() == EXIT_SUCCESS);
    CHECK(test_master_response_validation() == EXIT_SUCCESS);
    CHECK(test_transaction_engine() == EXIT_SUCCESS);

    puts("Modbus FC21 Write File Record tests: PASS");
    return EXIT_SUCCESS;
}
