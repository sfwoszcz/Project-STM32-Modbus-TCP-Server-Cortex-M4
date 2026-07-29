#include "modbus.h"
#include "modbus_crc16.h"
#include "modbus_fifo.h"
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

static uint16_t empty_fifo[1] = {0u};
static uint16_t example_fifo[3] = {2u, 0x01B8u, 0x1284u};
static uint16_t one_fifo[2] = {1u, 0xABCDu};
static uint16_t max_fifo[MB_FIFO_MAX_REGISTERS];
static uint16_t overflow_fifo[MB_FIFO_MAX_REGISTERS];
static uint16_t short_storage[2] = {2u, 0x1111u};

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

static int configure_standard_fifos(void)
{
    const mb_fifo_queue_t queues[] = {
        {0x0000u, empty_fifo, sizeof(empty_fifo) / sizeof(empty_fifo[0])},
        {0x0001u, one_fifo, sizeof(one_fifo) / sizeof(one_fifo[0])},
        {0x04DEu, example_fifo,
         sizeof(example_fifo) / sizeof(example_fifo[0])},
        {0x1000u, max_fifo, sizeof(max_fifo) / sizeof(max_fifo[0])}
    };

    max_fifo[0] = MB_FIFO_MAX_VALUES;
    for (uint16_t i = 0u; i < MB_FIFO_MAX_VALUES; ++i) {
        max_fifo[1u + (size_t)i] = (uint16_t)(0x1000u + i);
    }
    mb_init();
    return mb_fifo_configure(queues, sizeof(queues) / sizeof(queues[0]));
}

static int test_configuration_guards(void)
{
    mb_fifo_queue_t valid = {0x1234u, one_fifo, 2u};
    mb_fifo_queue_t too_many[MB_FIFO_MAX_QUEUES + 1u];
    mb_fifo_queue_t invalid;
    const uint8_t request[] = {MB_FIFO_FUNCTION_CODE, 0x12u, 0x34u};
    const uint8_t expected[] = {
        MB_FIFO_FUNCTION_CODE, 0x00u, 0x04u, 0x00u, 0x01u, 0xABu, 0xCDu
    };
    const uint8_t address_error[] = {0x98u, 0x02u};

    mb_init();
    CHECK(mb_fifo_is_configured() == 0);
    CHECK(mb_fifo_configure(NULL, 1u) == MB_FIFO_ERROR_ARGUMENT);
    CHECK(mb_fifo_configure(&valid, 0u) == MB_FIFO_ERROR_CAPACITY);
    CHECK(mb_fifo_configure(too_many, MB_FIFO_MAX_QUEUES + 1u) ==
          MB_FIFO_ERROR_CAPACITY);

    invalid = valid;
    invalid.registers = NULL;
    CHECK(mb_fifo_configure(&invalid, 1u) == MB_FIFO_ERROR_QUEUES);
    invalid = valid;
    invalid.register_count = 0u;
    CHECK(mb_fifo_configure(&invalid, 1u) == MB_FIFO_ERROR_QUEUES);
    invalid = valid;
    invalid.register_count = MB_FIFO_MAX_REGISTERS + 1u;
    CHECK(mb_fifo_configure(&invalid, 1u) == MB_FIFO_ERROR_QUEUES);

    {
        const mb_fifo_queue_t duplicate[] = {
            {1u, one_fifo, 2u}, {1u, one_fifo, 2u}
        };
        const mb_fifo_queue_t descending[] = {
            {2u, one_fifo, 2u}, {1u, one_fifo, 2u}
        };

        CHECK(mb_fifo_configure(duplicate, 2u) == MB_FIFO_ERROR_QUEUES);
        CHECK(mb_fifo_configure(descending, 2u) == MB_FIFO_ERROR_QUEUES);
    }

    CHECK(mb_fifo_configure(&valid, 1u) == MB_FIFO_OK);
    CHECK(mb_fifo_is_configured() != 0);
    CHECK(process_pdu_expect(request, sizeof(request), expected,
                             sizeof(expected)) == EXIT_SUCCESS);

    invalid = valid;
    invalid.registers = NULL;
    CHECK(mb_fifo_configure(&invalid, 1u) == MB_FIFO_ERROR_QUEUES);
    CHECK(process_pdu_expect(request, sizeof(request), expected,
                             sizeof(expected)) == EXIT_SUCCESS);

    one_fifo[1] = 0x5678u;
    {
        const uint8_t updated[] = {
            MB_FIFO_FUNCTION_CODE, 0x00u, 0x04u,
            0x00u, 0x01u, 0x56u, 0x78u
        };
        CHECK(process_pdu_expect(request, sizeof(request), updated,
                                 sizeof(updated)) == EXIT_SUCCESS);
    }
    one_fifo[1] = 0xABCDu;

    mb_fifo_clear();
    CHECK(mb_fifo_is_configured() == 0);
    CHECK(process_pdu_expect(request, sizeof(request), address_error,
                             sizeof(address_error)) == EXIT_SUCCESS);
    CHECK(mb_fifo_configure(&valid, 1u) == MB_FIFO_OK);
    mb_init();
    CHECK(mb_fifo_is_configured() == 0);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_example_and_non_destructive_read(void)
{
    const uint8_t request[] = {MB_FIFO_FUNCTION_CODE, 0x04u, 0xDEu};
    const uint8_t expected[] = {
        MB_FIFO_FUNCTION_CODE, 0x00u, 0x06u,
        0x00u, 0x02u, 0x01u, 0xB8u, 0x12u, 0x84u
    };
    uint16_t before[3];

    CHECK(configure_standard_fifos() == MB_FIFO_OK);
    memcpy(before, example_fifo, sizeof(before));
    CHECK(process_pdu_expect(request, sizeof(request), expected,
                             sizeof(expected)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(request, sizeof(request), expected,
                             sizeof(expected)) == EXIT_SUCCESS);
    CHECK(memcmp(before, example_fifo, sizeof(before)) == 0);
    return EXIT_SUCCESS;
}

static int test_empty_one_max_and_validation(void)
{
    const uint8_t empty_request[] = {MB_FIFO_FUNCTION_CODE, 0x00u, 0x00u};
    const uint8_t empty_response[] = {
        MB_FIFO_FUNCTION_CODE, 0x00u, 0x02u, 0x00u, 0x00u
    };
    const uint8_t one_request[] = {MB_FIFO_FUNCTION_CODE, 0x00u, 0x01u};
    const uint8_t one_response[] = {
        MB_FIFO_FUNCTION_CODE, 0x00u, 0x04u,
        0x00u, 0x01u, 0xABu, 0xCDu
    };
    const uint8_t max_request[] = {MB_FIFO_FUNCTION_CODE, 0x10u, 0x00u};
    const uint8_t short_request[] = {MB_FIFO_FUNCTION_CODE, 0x10u};
    const uint8_t long_request[] = {
        MB_FIFO_FUNCTION_CODE, 0x10u, 0x00u, 0x00u
    };
    const uint8_t unknown_request[] = {
        MB_FIFO_FUNCTION_CODE, 0xFFu, 0xFFu
    };
    const uint8_t value_error[] = {0x98u, 0x03u};
    const uint8_t address_error[] = {0x98u, 0x02u};
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    CHECK(configure_standard_fifos() == MB_FIFO_OK);
    CHECK(process_pdu_expect(empty_request, sizeof(empty_request),
                             empty_response, sizeof(empty_response)) ==
          EXIT_SUCCESS);
    CHECK(process_pdu_expect(one_request, sizeof(one_request), one_response,
                             sizeof(one_response)) == EXIT_SUCCESS);

    CHECK(mb_process_pdu(max_request,
                         sizeof(max_request),
                         response,
                         sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == MB_FIFO_MAX_RESPONSE_PDU_SIZE);
    CHECK(response[0] == MB_FIFO_FUNCTION_CODE);
    CHECK(response[1] == 0u && response[2] == MB_FIFO_MAX_BYTE_COUNT);
    CHECK(response[3] == 0u && response[4] == MB_FIFO_MAX_VALUES);
    for (uint16_t i = 0u; i < MB_FIFO_MAX_VALUES; ++i) {
        size_t offset = 5u + ((size_t)i * 2u);
        CHECK(response[offset] == (uint8_t)((0x1000u + i) >> 8u));
        CHECK(response[offset + 1u] == (uint8_t)(0x1000u + i));
    }

    CHECK(process_pdu_expect(short_request, sizeof(short_request),
                             value_error, sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(long_request, sizeof(long_request),
                             value_error, sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(unknown_request, sizeof(unknown_request),
                             address_error, sizeof(address_error)) ==
          EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static int test_count_and_storage_failures(void)
{
    const mb_fifo_queue_t queues[] = {
        {0x2000u, overflow_fifo,
         sizeof(overflow_fifo) / sizeof(overflow_fifo[0])},
        {0x2001u, short_storage,
         sizeof(short_storage) / sizeof(short_storage[0])}
    };
    const uint8_t overflow_request[] = {
        MB_FIFO_FUNCTION_CODE, 0x20u, 0x00u
    };
    const uint8_t storage_request[] = {
        MB_FIFO_FUNCTION_CODE, 0x20u, 0x01u
    };
    const uint8_t value_error[] = {0x98u, 0x03u};
    const uint8_t server_error[] = {0x98u, 0x04u};
    uint8_t response[2];
    size_t response_length = 0u;

    overflow_fifo[0] = MB_FIFO_MAX_VALUES + 1u;
    mb_init();
    CHECK(mb_fifo_configure(queues, 2u) == MB_FIFO_OK);
    CHECK(process_pdu_expect(overflow_request, sizeof(overflow_request),
                             value_error, sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(storage_request, sizeof(storage_request),
                             server_error, sizeof(server_error)) == EXIT_SUCCESS);

    CHECK(configure_standard_fifos() == MB_FIFO_OK);
    CHECK(mb_process_pdu((const uint8_t[]){MB_FIFO_FUNCTION_CODE, 0x04u, 0xDEu},
                         3u,
                         response,
                         sizeof(response),
                         &response_length) == 0);
    CHECK(response_length == 2u);
    CHECK(response[0] == 0x98u && response[1] == 0x04u);
    return EXIT_SUCCESS;
}

static int test_tcp_rtu_and_broadcast_paths(void)
{
    const uint8_t pdu[] = {MB_FIFO_FUNCTION_CODE, 0x04u, 0xDEu};
    const uint8_t expected_pdu[] = {
        MB_FIFO_FUNCTION_CODE, 0x00u, 0x06u,
        0x00u, 0x02u, 0x01u, 0xB8u, 0x12u, 0x84u
    };
    uint8_t request[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    CHECK(configure_standard_fifos() == MB_FIFO_OK);
    request_length = make_tcp_adu(pdu, sizeof(pdu), request);
    CHECK(mbtcp_process_adu(request,
                            request_length,
                            response,
                            sizeof(response),
                            &response_length) == 0);
    CHECK(response_length == MBTCP_MBAP_HEADER_SIZE + sizeof(expected_pdu));
    CHECK(memcmp(&response[7], expected_pdu, sizeof(expected_pdu)) == 0);

    request_length = make_rtu_adu(1u, pdu, sizeof(pdu), request);
    response_length = 0u;
    CHECK(mbrtu_process_adu(1u,
                            request,
                            request_length,
                            response,
                            sizeof(response),
                            &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response_length == 1u + sizeof(expected_pdu) + MODBUS_RTU_CRC_SIZE);
    CHECK(memcmp(&response[1], expected_pdu, sizeof(expected_pdu)) == 0);
    CHECK(mb_crc16(response, response_length) == 0u);

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
    return EXIT_SUCCESS;
}

static int test_master_builder_and_decoder(void)
{
    const uint8_t expected_without_crc[] = {
        1u, MBRTUM_FC_READ_FIFO_QUEUE, 0x04u, 0xDEu
    };
    mbrtum_request_t request;
    mbrtum_response_t response;
    mbrtum_fifo_response_t fifo_response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE] = {
        1u, MBRTUM_FC_READ_FIFO_QUEUE,
        0x00u, 0x06u, 0x00u, 0x02u, 0x01u, 0xB8u, 0x12u, 0x84u
    };
    size_t request_length = 99u;
    size_t response_length;
    uint16_t value = 0u;

    CHECK(mbrtum_build_read_fifo_queue_request(
              1u, 0x04DEu, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    CHECK(request_length == 6u);
    CHECK(memcmp(request_adu, expected_without_crc,
                 sizeof(expected_without_crc)) == 0);
    CHECK(mb_crc16(request_adu, request_length) == 0u);
    CHECK(request.slave_address == 1u);
    CHECK(request.function == MBRTUM_FC_READ_FIFO_QUEUE);
    CHECK(request.start_address == 0x04DEu);
    CHECK(request.quantity == 0u && request.value == 0u);
    CHECK(request.expects_response == 1u);
    CHECK(request.write_start_address == 0u && request.write_quantity == 0u);

    request_length = 99u;
    memset(&request, 0xA5, sizeof(request));
    CHECK(mbrtum_build_read_fifo_queue_request(
              0u, 0u, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(request_length == 0u && request.function == 0u);
    CHECK(mbrtum_build_read_fifo_queue_request(
              248u, 0u, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(mbrtum_build_read_fifo_queue_request(
              1u, 0u, &request, request_adu, 5u,
              &request_length) == MBRTUM_ERROR_CAPACITY);

    CHECK(mbrtum_build_read_fifo_queue_request(
              1u, 0x04DEu, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    response_length = append_crc(response_adu, 10u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.function == MBRTUM_FC_READ_FIFO_QUEUE);
    CHECK(response.data == &response_adu[4]);
    CHECK(response.data_length == 6u);
    CHECK(mbrtum_get_fifo_response(&response, &fifo_response) == MBRTUM_OK);
    CHECK(fifo_response.fifo_count == 2u);
    CHECK(fifo_response.values == &response_adu[6]);
    CHECK(fifo_response.values_length == 4u);
    CHECK(mbrtum_get_fifo_register(&fifo_response, 0u, &value) == MBRTUM_OK);
    CHECK(value == 0x01B8u);
    CHECK(mbrtum_get_fifo_register(&fifo_response, 1u, &value) == MBRTUM_OK);
    CHECK(value == 0x1284u);
    CHECK(mbrtum_get_fifo_register(&fifo_response, 2u, &value) ==
          MBRTUM_ERROR_INDEX);
    return EXIT_SUCCESS;
}

static int test_master_response_validation(void)
{
    mbrtum_request_t request;
    mbrtum_response_t response;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t valid[MODBUS_RTU_ADU_MAX_SIZE] = {
        1u, MBRTUM_FC_READ_FIFO_QUEUE,
        0x00u, 0x06u, 0x00u, 0x02u, 0x01u, 0xB8u, 0x12u, 0x84u
    };
    uint8_t malformed[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t exception[] = {1u, 0x98u, 0x02u, 0u, 0u};
    uint8_t empty[] = {
        1u, MBRTUM_FC_READ_FIFO_QUEUE, 0x00u, 0x02u, 0x00u, 0x00u,
        0u, 0u
    };
    uint8_t too_many[72] = {
        1u, MBRTUM_FC_READ_FIFO_QUEUE, 0x00u, 0x42u, 0x00u, 0x20u
    };
    mbrtum_fifo_response_t fifo_response;
    size_t request_length = 0u;
    size_t valid_length;
    size_t empty_length;
    size_t too_many_length;

    CHECK(mbrtum_build_read_fifo_queue_request(
              1u, 0x04DEu, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    valid_length = append_crc(valid, 10u);

    empty_length = append_crc(empty, 6u);
    CHECK(mbrtum_process_response(&request, empty, empty_length,
                                  &response) == MBRTUM_OK);
    CHECK(mbrtum_get_fifo_response(&response, &fifo_response) == MBRTUM_OK);
    CHECK(fifo_response.fifo_count == 0u);
    CHECK(fifo_response.values == NULL);
    CHECK(fifo_response.values_length == 0u);

    too_many_length = append_crc(too_many, 70u);
    CHECK(mbrtum_process_response(&request, too_many, too_many_length,
                                  &response) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);

    memcpy(malformed, valid, valid_length);
    malformed[valid_length - 1u] ^= 1u;
    CHECK(mbrtum_process_response(&request, malformed, valid_length,
                                  &response) == MBRTUM_ERROR_CRC);

    memcpy(malformed, valid, valid_length);
    malformed[0] = 2u;
    replace_crc(malformed, valid_length);
    CHECK(mbrtum_process_response(&request, malformed, valid_length,
                                  &response) == MBRTUM_ERROR_ADDRESS_MISMATCH);

    memcpy(malformed, valid, valid_length);
    malformed[1] = 0x17u;
    replace_crc(malformed, valid_length);
    CHECK(mbrtum_process_response(&request, malformed, valid_length,
                                  &response) == MBRTUM_ERROR_FUNCTION_MISMATCH);

    memcpy(malformed, valid, valid_length);
    malformed[3] = 0x04u;
    replace_crc(malformed, valid_length);
    CHECK(mbrtum_process_response(&request, malformed, valid_length,
                                  &response) == MBRTUM_ERROR_MALFORMED_RESPONSE);

    memcpy(malformed, valid, valid_length);
    malformed[5] = 0x01u;
    replace_crc(malformed, valid_length);
    CHECK(mbrtum_process_response(&request, malformed, valid_length,
                                  &response) == MBRTUM_ERROR_MALFORMED_RESPONSE);

    CHECK(mbrtum_process_response(&request, valid, valid_length - 1u,
                                  &response) == MBRTUM_ERROR_CRC);

    exception[3] = (uint8_t)mb_crc16(exception, 3u);
    exception[4] = (uint8_t)(mb_crc16(exception, 3u) >> 8u);
    CHECK(mbrtum_process_response(&request, exception, sizeof(exception),
                                  &response) == MBRTUM_EXCEPTION_RESPONSE);
    CHECK(response.exception_code == 0x02u);

    request.quantity = 1u;
    CHECK(mbrtum_process_response(&request, valid, valid_length,
                                  &response) == MBRTUM_ERROR_VALUE);
    return EXIT_SUCCESS;
}

static int transmit_accept(void *context, const uint8_t *adu, size_t adu_length)
{
    size_t *last_length = (size_t *)context;

    (void)adu;
    *last_length = adu_length;
    return MBRTUM_TXN_TRANSMIT_ACCEPTED;
}

static int test_transaction_engine(void)
{
    const mbrtum_transaction_config_t config = {
        10u, 1u, 0u, 0u
    };
    mbrtum_transaction_t transaction;
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE] = {
        1u, MBRTUM_FC_READ_FIFO_QUEUE,
        0x00u, 0x06u, 0x00u, 0x02u, 0x01u, 0xB8u, 0x12u, 0x84u
    };
    size_t request_length = 0u;
    size_t response_length;
    size_t transmitted_length = 0u;

    CHECK(mbrtum_build_read_fifo_queue_request(
              1u, 0x04DEu, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&transaction, &config, transmit_accept,
                                  &transmitted_length) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 100u) ==
          MBRTUM_TXN_OK);
    CHECK(transmitted_length == request_length);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 101u) ==
          MBRTUM_TXN_OK);
    response_length = append_crc(response_adu, 10u);
    CHECK(mbrtum_transaction_on_response(&transaction,
                                         response_adu,
                                         response_length,
                                         102u) == MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(transaction.protocol_result == MBRTUM_OK);

    CHECK(mbrtum_build_read_fifo_queue_request(
              1u, 0x04DEu, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    request_adu[3] ^= 1u;
    replace_crc(request_adu, request_length);
    CHECK(mbrtum_transaction_start(&transaction, &request,
                                   request_adu, request_length, 200u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_configuration_guards() == EXIT_SUCCESS);
    CHECK(test_shared_pdu_example_and_non_destructive_read() == EXIT_SUCCESS);
    CHECK(test_empty_one_max_and_validation() == EXIT_SUCCESS);
    CHECK(test_count_and_storage_failures() == EXIT_SUCCESS);
    CHECK(test_tcp_rtu_and_broadcast_paths() == EXIT_SUCCESS);
    CHECK(test_master_builder_and_decoder() == EXIT_SUCCESS);
    CHECK(test_master_response_validation() == EXIT_SUCCESS);
    CHECK(test_transaction_engine() == EXIT_SUCCESS);

    puts("Modbus FC24 Read FIFO Queue tests: PASS");
    return EXIT_SUCCESS;
}
