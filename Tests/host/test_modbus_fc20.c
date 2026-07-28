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

static uint16_t file3[16];
static uint16_t file4[16];
static uint16_t large_file[MB_FILE_RECORD_MAX_RECORDS_PER_FILE];

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

static int configure_example_files(void)
{
    const mb_file_record_file_t files[] = {
        {3u, file3, sizeof(file3) / sizeof(file3[0])},
        {4u, file4, sizeof(file4) / sizeof(file4[0])}
    };

    memset(file3, 0, sizeof(file3));
    memset(file4, 0, sizeof(file4));
    file4[1] = 0x0DFEu;
    file4[2] = 0x0020u;
    file3[9] = 0x33CDu;
    file3[10] = 0x0040u;
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

static int test_configuration_guards(void)
{
    uint16_t value = 0x1234u;
    mb_file_record_file_t valid = {1u, &value, 1u};
    mb_file_record_file_t too_many[MB_FILE_RECORD_MAX_FILES + 1u];
    mb_file_record_file_t invalid;
    const uint8_t request[] = {
        0x14u, 0x07u, 0x06u, 0x00u, 0x01u,
        0x00u, 0x00u, 0x00u, 0x01u
    };
    const uint8_t expected[] = {0x14u, 0x04u, 0x03u, 0x06u, 0x12u, 0x34u};
    const uint8_t address_error[] = {0x94u, 0x02u};

    mb_init();
    CHECK(mb_file_record_is_configured() == 0);
    CHECK(mb_file_record_configure(NULL, 1u) ==
          MB_FILE_RECORD_ERROR_ARGUMENT);
    CHECK(mb_file_record_configure(&valid, 0u) ==
          MB_FILE_RECORD_ERROR_CAPACITY);
    CHECK(mb_file_record_configure(too_many,
                                   MB_FILE_RECORD_MAX_FILES + 1u) ==
          MB_FILE_RECORD_ERROR_CAPACITY);

    invalid = valid;
    invalid.file_number = 0u;
    CHECK(mb_file_record_configure(&invalid, 1u) ==
          MB_FILE_RECORD_ERROR_FILES);
    invalid = valid;
    invalid.records = NULL;
    CHECK(mb_file_record_configure(&invalid, 1u) ==
          MB_FILE_RECORD_ERROR_FILES);
    invalid = valid;
    invalid.record_count = 0u;
    CHECK(mb_file_record_configure(&invalid, 1u) ==
          MB_FILE_RECORD_ERROR_FILES);
    invalid = valid;
    invalid.record_count = MB_FILE_RECORD_MAX_RECORDS_PER_FILE + 1u;
    CHECK(mb_file_record_configure(&invalid, 1u) ==
          MB_FILE_RECORD_ERROR_FILES);

    {
        mb_file_record_file_t duplicate[] = {
            {1u, &value, 1u}, {1u, &value, 1u}
        };
        mb_file_record_file_t descending[] = {
            {2u, &value, 1u}, {1u, &value, 1u}
        };

        CHECK(mb_file_record_configure(duplicate, 2u) ==
              MB_FILE_RECORD_ERROR_FILES);
        CHECK(mb_file_record_configure(descending, 2u) ==
              MB_FILE_RECORD_ERROR_FILES);
    }

    CHECK(mb_file_record_configure(&valid, 1u) == MB_FILE_RECORD_OK);
    CHECK(mb_file_record_is_configured() != 0);
    CHECK(process_pdu_expect(request, sizeof(request), expected,
                             sizeof(expected)) == EXIT_SUCCESS);

    invalid = valid;
    invalid.file_number = 0u;
    CHECK(mb_file_record_configure(&invalid, 1u) ==
          MB_FILE_RECORD_ERROR_FILES);
    CHECK(process_pdu_expect(request, sizeof(request), expected,
                             sizeof(expected)) == EXIT_SUCCESS);

    value = 0x5678u;
    {
        const uint8_t updated[] = {
            0x14u, 0x04u, 0x03u, 0x06u, 0x56u, 0x78u
        };
        CHECK(process_pdu_expect(request, sizeof(request), updated,
                                 sizeof(updated)) == EXIT_SUCCESS);
    }

    mb_file_record_clear();
    CHECK(mb_file_record_is_configured() == 0);
    CHECK(process_pdu_expect(request, sizeof(request), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    CHECK(mb_file_record_configure(&valid, 1u) == MB_FILE_RECORD_OK);
    mb_init();
    CHECK(mb_file_record_is_configured() == 0);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_example_and_validation(void)
{
    const uint8_t request[] = {
        0x14u, 0x0Eu,
        0x06u, 0x00u, 0x04u, 0x00u, 0x01u, 0x00u, 0x02u,
        0x06u, 0x00u, 0x03u, 0x00u, 0x09u, 0x00u, 0x02u
    };
    const uint8_t expected[] = {
        0x14u, 0x0Cu,
        0x05u, 0x06u, 0x0Du, 0xFEu, 0x00u, 0x20u,
        0x05u, 0x06u, 0x33u, 0xCDu, 0x00u, 0x40u
    };
    const uint8_t value_error[] = {0x94u, 0x03u};
    const uint8_t address_error[] = {0x94u, 0x02u};
    uint8_t malformed[sizeof(request)];
    uint8_t response[2];
    size_t response_length = 0u;

    CHECK(configure_example_files() == MB_FILE_RECORD_OK);
    CHECK(process_pdu_expect(request, sizeof(request), expected,
                             sizeof(expected)) == EXIT_SUCCESS);

    memcpy(malformed, request, sizeof(malformed));
    malformed[1] = 6u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(malformed));
    malformed[1] = 8u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(request, sizeof(request) - 1u, value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);

    memcpy(malformed, request, sizeof(malformed));
    malformed[2] = 5u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(malformed));
    malformed[3] = 0u;
    malformed[4] = 0u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(malformed));
    malformed[4] = 5u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(malformed));
    malformed[5] = 0x27u;
    malformed[6] = 0x10u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(malformed));
    malformed[7] = 0u;
    malformed[8] = 0u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    memcpy(malformed, request, sizeof(malformed));
    malformed[5] = 0u;
    malformed[6] = 15u;
    malformed[7] = 0u;
    malformed[8] = 2u;
    CHECK(process_pdu_expect(malformed, sizeof(malformed), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);

    CHECK(mb_process_pdu(request, sizeof(request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response_length == 2u);
    CHECK(response[0] == 0x94u && response[1] == 0x04u);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_boundaries(void)
{
    mb_file_record_file_t file = {
        1u, large_file, sizeof(large_file) / sizeof(large_file[0])
    };
    uint8_t request[2u + MB_FILE_RECORD_REQUEST_DATA_MAX];
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    for (size_t i = 0u; i < sizeof(large_file) / sizeof(large_file[0]); ++i) {
        large_file[i] = (uint16_t)i;
    }
    mb_init();
    CHECK(mb_file_record_configure(&file, 1u) == MB_FILE_RECORD_OK);

    memset(request, 0, sizeof(request));
    request[0] = 0x14u;
    request[1] = 7u;
    request[2] = 6u;
    request[4] = 1u;
    request[7] = 0u;
    request[8] = 121u;
    CHECK(mb_process_pdu(request, 9u, response, sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == 246u);
    CHECK(response[1] == 244u);
    CHECK(response[2] == 243u && response[3] == 6u);
    CHECK(response[4] == 0u && response[5] == 0u);
    CHECK(response[244] == 0u && response[245] == 120u);

    request[8] = 122u;
    {
        const uint8_t expected[] = {0x94u, 0x03u};
        CHECK(process_pdu_expect(request, 9u, expected,
                                 sizeof(expected)) == EXIT_SUCCESS);
    }

    request[1] = MB_FILE_RECORD_REQUEST_DATA_MAX;
    for (size_t i = 0u; i < MB_FILE_RECORD_MAX_SUBREQUESTS; ++i) {
        size_t offset = 2u + (i * 7u);

        request[offset] = 6u;
        request[offset + 1u] = 0u;
        request[offset + 2u] = 1u;
        request[offset + 3u] = 0u;
        request[offset + 4u] = (uint8_t)i;
        request[offset + 5u] = 0u;
        request[offset + 6u] = 1u;
    }
    CHECK(mb_process_pdu(request, sizeof(request), response, sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == 142u);
    CHECK(response[1] == 140u);
    return EXIT_SUCCESS;
}

static int test_tcp_and_rtu_paths(void)
{
    const uint8_t request_pdu[] = {
        0x14u, 0x0Eu,
        0x06u, 0x00u, 0x04u, 0x00u, 0x01u, 0x00u, 0x02u,
        0x06u, 0x00u, 0x03u, 0x00u, 0x09u, 0x00u, 0x02u
    };
    const uint8_t response_pdu[] = {
        0x14u, 0x0Cu,
        0x05u, 0x06u, 0x0Du, 0xFEu, 0x00u, 0x20u,
        0x05u, 0x06u, 0x33u, 0xCDu, 0x00u, 0x40u
    };
    uint8_t request_adu[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_TCP_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    CHECK(configure_example_files() == MB_FILE_RECORD_OK);
    request_length = make_tcp_adu(request_pdu, sizeof(request_pdu), request_adu);
    CHECK(mbtcp_process_adu(request_adu, request_length, response_adu,
                            sizeof(response_adu), &response_length) == 0);
    CHECK(response_length == MBTCP_MBAP_HEADER_SIZE + sizeof(response_pdu));
    CHECK(memcmp(&response_adu[7], response_pdu, sizeof(response_pdu)) == 0);

    request_length = make_rtu_adu(1u, request_pdu, sizeof(request_pdu),
                                  request_adu);
    CHECK(mbrtu_process_adu(1u, request_adu, request_length,
                            response_adu, sizeof(response_adu),
                            &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response_length == 1u + sizeof(response_pdu) + MODBUS_RTU_CRC_SIZE);
    CHECK(response_adu[0] == 1u);
    CHECK(memcmp(&response_adu[1], response_pdu, sizeof(response_pdu)) == 0);
    CHECK(mb_crc16(response_adu, response_length) == 0u);
    return EXIT_SUCCESS;
}

static int test_master_builder(void)
{
    const mbrtum_file_record_request_t subrequests[] = {
        {4u, 1u, 2u}, {3u, 9u, 2u}
    };
    const uint8_t expected_without_crc[] = {
        1u, 0x14u, 0x0Eu,
        0x06u, 0x00u, 0x04u, 0x00u, 0x01u, 0x00u, 0x02u,
        0x06u, 0x00u, 0x03u, 0x00u, 0x09u, 0x00u, 0x02u
    };
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    CHECK(mbrtum_build_read_file_record_request(
              1u, subrequests, 2u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_OK);
    CHECK(adu_length == sizeof(expected_without_crc) + MODBUS_RTU_CRC_SIZE);
    CHECK(memcmp(adu, expected_without_crc, sizeof(expected_without_crc)) == 0);
    CHECK(mb_crc16(adu, adu_length) == 0u);
    CHECK(request.function == MBRTUM_FC_READ_FILE_RECORD);
    CHECK(request.start_address == 14u);
    CHECK(request.quantity == 12u);
    CHECK(request.value == 2u);
    CHECK(request.expects_response == 1u);

    CHECK(mbrtum_build_read_file_record_request(
              0u, subrequests, 2u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(mbrtum_build_read_file_record_request(
              1u, NULL, 2u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_ARGUMENT);
    CHECK(mbrtum_build_read_file_record_request(
              1u, subrequests, 0u, &request, adu, sizeof(adu),
              &adu_length) == MBRTUM_ERROR_QUANTITY);
    CHECK(mbrtum_build_read_file_record_request(
              1u, subrequests, 2u, &request, adu, 18u,
              &adu_length) == MBRTUM_ERROR_CAPACITY);

    {
        mbrtum_file_record_request_t invalid = {0u, 0u, 1u};
        CHECK(mbrtum_build_read_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_VALUE);
        invalid.file_number = 1u;
        invalid.record_number = 10000u;
        CHECK(mbrtum_build_read_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_VALUE);
        invalid.record_number = 0u;
        invalid.record_length = 0u;
        CHECK(mbrtum_build_read_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_VALUE);
        invalid.record_length = 122u;
        CHECK(mbrtum_build_read_file_record_request(
                  1u, &invalid, 1u, &request, adu, sizeof(adu),
                  &adu_length) == MBRTUM_ERROR_QUANTITY);
    }
    return EXIT_SUCCESS;
}

static int test_master_maximum_request(void)
{
    mbrtum_file_record_request_t
        subrequests[MB_FILE_RECORD_MAX_SUBREQUESTS];
    mbrtum_request_t request;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t adu_length = 0u;

    for (size_t i = 0u; i < MB_FILE_RECORD_MAX_SUBREQUESTS; ++i) {
        subrequests[i].file_number = 1u;
        subrequests[i].record_number = (uint16_t)i;
        subrequests[i].record_length = 1u;
    }
    CHECK(mbrtum_build_read_file_record_request(
              247u, subrequests, MB_FILE_RECORD_MAX_SUBREQUESTS,
              &request, adu, sizeof(adu), &adu_length) == MBRTUM_OK);
    CHECK(adu_length == 250u);
    CHECK(adu[2] == 245u);
    CHECK(request.quantity == 140u);
    CHECK(mb_crc16(adu, adu_length) == 0u);
    return EXIT_SUCCESS;
}

static int test_master_response_and_decoders(void)
{
    const mbrtum_file_record_request_t subrequests[] = {
        {4u, 1u, 2u}, {3u, 9u, 2u}
    };
    const uint8_t response_pdu[] = {
        0x14u, 0x0Cu,
        0x05u, 0x06u, 0x0Du, 0xFEu, 0x00u, 0x20u,
        0x05u, 0x06u, 0x33u, 0xCDu, 0x00u, 0x40u
    };
    mbrtum_request_t request;
    mbrtum_response_t response;
    mbrtum_file_record_response_t file_response;
    mbrtum_file_record_subresponse_t subresponse;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;
    uint16_t value = 0u;

    CHECK(mbrtum_build_read_file_record_request(
              1u, subrequests, 2u, &request, request_adu,
              sizeof(request_adu), &request_length) == MBRTUM_OK);
    response_length = make_rtu_adu(1u, response_pdu,
                                   sizeof(response_pdu), response_adu);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) ==
          MBRTUM_ERROR_REQUEST_DATA_REQUIRED);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, response_length, &response) == MBRTUM_OK);
    CHECK(response.function == MBRTUM_FC_READ_FILE_RECORD);
    CHECK(response.data_length == 12u);
    CHECK(mbrtum_get_file_record_response(&response, &file_response) ==
          MBRTUM_OK);
    CHECK(file_response.subresponse_count == 2u);
    CHECK(mbrtum_get_file_record_subresponse(&file_response, 0u,
                                              &subresponse) == MBRTUM_OK);
    CHECK(subresponse.record_length == 2u);
    CHECK(mbrtum_get_file_record_register(&subresponse, 0u, &value) ==
          MBRTUM_OK);
    CHECK(value == 0x0DFEu);
    CHECK(mbrtum_get_file_record_register(&subresponse, 1u, &value) ==
          MBRTUM_OK);
    CHECK(value == 0x0020u);
    CHECK(mbrtum_get_file_record_subresponse(&file_response, 1u,
                                              &subresponse) == MBRTUM_OK);
    CHECK(mbrtum_get_file_record_register(&subresponse, 0u, &value) ==
          MBRTUM_OK);
    CHECK(value == 0x33CDu);
    CHECK(mbrtum_get_file_record_register(&subresponse, 2u, &value) ==
          MBRTUM_ERROR_INDEX);
    CHECK(mbrtum_get_file_record_subresponse(&file_response, 2u,
                                              &subresponse) ==
          MBRTUM_ERROR_INDEX);

    response_adu[2] = 10u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, response_length, &response) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);
    response_adu[2] = 12u;
    response_adu[3] = 3u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, response_length, &response) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);
    response_adu[3] = 5u;
    response_adu[4] = 5u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, response_length, &response) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);
    response_adu[4] = 6u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, response_length - 1u, &response) ==
          MBRTUM_ERROR_CRC);

    request_adu[3] = 5u;
    replace_crc(request_adu, request_length);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, response_length, &response) ==
          MBRTUM_ERROR_REQUEST_DATA_REQUIRED);
    return EXIT_SUCCESS;
}

static int test_master_exception_response(void)
{
    const mbrtum_file_record_request_t subrequest = {1u, 0u, 1u};
    const uint8_t exception_pdu[] = {0x94u, 0x02u};
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;

    CHECK(mbrtum_build_read_file_record_request(
              1u, &subrequest, 1u, &request, request_adu,
              sizeof(request_adu), &request_length) == MBRTUM_OK);
    response_length = make_rtu_adu(1u, exception_pdu,
                                   sizeof(exception_pdu), response_adu);
    CHECK(mbrtum_process_response_with_request_adu(
              &request, request_adu, request_length,
              response_adu, response_length, &response) ==
          MBRTUM_EXCEPTION_RESPONSE);
    CHECK(response.exception_code == 0x02u);
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
    const mbrtum_file_record_request_t subrequests[] = {
        {4u, 1u, 2u}, {3u, 9u, 2u}
    };
    const uint8_t response_pdu[] = {
        0x14u, 0x0Cu,
        0x05u, 0x06u, 0x0Du, 0xFEu, 0x00u, 0x20u,
        0x05u, 0x06u, 0x33u, 0xCDu, 0x00u, 0x40u
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
    mbrtum_file_record_response_t file_response;
    mbrtum_file_record_subresponse_t subresponse;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;
    uint16_t value = 0u;

    CHECK(mbrtum_build_read_file_record_request(
              1u, subrequests, 2u, &request, request_adu,
              sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&transaction, &config,
                                  fake_transmit, &transport) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 100u) ==
          MBRTUM_TXN_OK);
    CHECK(transport.calls == 1u);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 101u) ==
          MBRTUM_TXN_OK);
    response_length = make_rtu_adu(1u, response_pdu,
                                   sizeof(response_pdu), response_adu);
    CHECK(mbrtum_transaction_on_response(&transaction,
                                         response_adu,
                                         response_length,
                                         102u) == MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(transaction.protocol_result == MBRTUM_OK);
    CHECK(mbrtum_get_file_record_response(&transaction.response,
                                           &file_response) == MBRTUM_OK);
    CHECK(mbrtum_get_file_record_subresponse(&file_response, 1u,
                                              &subresponse) == MBRTUM_OK);
    CHECK(mbrtum_get_file_record_register(&subresponse, 1u, &value) ==
          MBRTUM_OK);
    CHECK(value == 0x0040u);

    request.quantity = 11u;
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 200u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_configuration_guards() == EXIT_SUCCESS);
    CHECK(test_shared_pdu_example_and_validation() == EXIT_SUCCESS);
    CHECK(test_shared_pdu_boundaries() == EXIT_SUCCESS);
    CHECK(test_tcp_and_rtu_paths() == EXIT_SUCCESS);
    CHECK(test_master_builder() == EXIT_SUCCESS);
    CHECK(test_master_maximum_request() == EXIT_SUCCESS);
    CHECK(test_master_response_and_decoders() == EXIT_SUCCESS);
    CHECK(test_master_exception_response() == EXIT_SUCCESS);
    CHECK(test_transaction_engine() == EXIT_SUCCESS);

    puts("Modbus FC20 Read File Record tests: PASS");
    return EXIT_SUCCESS;
}
