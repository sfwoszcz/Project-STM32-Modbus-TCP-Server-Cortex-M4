#include "modbus.h"
#include "modbus_crc16.h"
#include "modbus_device_id.h"
#include "modbus_mei.h"
#include "modbus_pdu.h"
#include "modbus_protocol.h"
#include "modbus_rtu.h"
#include "modbus_rtu_master.h"
#include "modbus_rtu_master_transaction.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_MEI_TYPE 0x80u

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

typedef struct {
    size_t calls;
    uint8_t exception_code;
    uint8_t xor_mask;
    uint8_t force_oversize;
} handler_context_t;

typedef struct {
    size_t calls;
    size_t length;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
} transport_t;

static uint8_t test_handler(void *context,
                            uint8_t mei_type,
                            const uint8_t *request_data,
                            size_t request_data_length,
                            uint8_t *response_data,
                            size_t response_capacity,
                            size_t *response_data_length)
{
    handler_context_t *state = (handler_context_t *)context;

    if (state == NULL || response_data == NULL || response_data_length == NULL ||
        mei_type != TEST_MEI_TYPE) {
        return MB_MEI_EXCEPTION_SERVER_FAILURE;
    }
    ++state->calls;
    *response_data_length = 0u;
    if (state->exception_code != 0u) {
        return state->exception_code;
    }
    if (state->force_oversize != 0u) {
        *response_data_length = response_capacity + 1u;
        return MB_MEI_HANDLER_OK;
    }
    if (request_data_length > response_capacity ||
        (request_data_length != 0u && request_data == NULL)) {
        return MB_MEI_EXCEPTION_SERVER_FAILURE;
    }
    for (size_t i = 0u; i < request_data_length; ++i) {
        response_data[i] = (uint8_t)(request_data[i] ^ state->xor_mask);
    }
    *response_data_length = request_data_length;
    return MB_MEI_HANDLER_OK;
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

static size_t make_rtu_adu(uint8_t address,
                           const uint8_t *pdu,
                           size_t pdu_length,
                           uint8_t *adu)
{
    adu[0] = address;
    memcpy(&adu[1], pdu, pdu_length);
    return append_crc(adu, 1u + pdu_length);
}

static int process_pdu_expect(const uint8_t *request,
                              size_t request_length,
                              size_t response_capacity,
                              const uint8_t *expected,
                              size_t expected_length)
{
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    CHECK(response_capacity <= sizeof(response));
    CHECK(mb_process_pdu(request,
                         request_length,
                         response,
                         response_capacity,
                         &response_length) == 0);
    CHECK(response_length == expected_length);
    CHECK(memcmp(response, expected, expected_length) == 0);
    return EXIT_SUCCESS;
}

static int fake_transmit(void *context,
                         const uint8_t *adu,
                         size_t adu_length)
{
    transport_t *transport = (transport_t *)context;

    if (transport == NULL || adu == NULL ||
        adu_length > sizeof(transport->adu)) {
        return -1;
    }
    ++transport->calls;
    transport->length = adu_length;
    memcpy(transport->adu, adu, adu_length);
    return MBRTUM_TXN_TRANSMIT_ACCEPTED;
}

static int test_registration_guards(void)
{
    handler_context_t handlers[MB_MEI_MAX_HANDLERS + 1u];

    memset(handlers, 0, sizeof(handlers));
    mb_init();
    CHECK(mb_mei_is_registered(TEST_MEI_TYPE) == 0);
    CHECK(mb_mei_register_handler(TEST_MEI_TYPE, NULL, NULL) ==
          MB_MEI_ERROR_ARGUMENT);
    CHECK(mb_mei_register_handler(MB_MEI_TYPE_CANOPEN_GENERAL_REFERENCE,
                                  test_handler, &handlers[0]) ==
          MB_MEI_ERROR_TYPE);
    CHECK(mb_mei_register_handler(MB_MEI_TYPE_READ_DEVICE_IDENTIFICATION,
                                  test_handler, &handlers[0]) ==
          MB_MEI_ERROR_TYPE);
    CHECK(mb_mei_unregister_handler(MB_MEI_TYPE_CANOPEN_GENERAL_REFERENCE) ==
          MB_MEI_ERROR_TYPE);

    for (size_t i = 0u; i < MB_MEI_MAX_HANDLERS; ++i) {
        CHECK(mb_mei_register_handler((uint8_t)(0x80u + i), test_handler,
                                      &handlers[i]) == MB_MEI_OK);
    }
    CHECK(mb_mei_register_handler(0xF0u, test_handler,
                                  &handlers[MB_MEI_MAX_HANDLERS]) ==
          MB_MEI_ERROR_CAPACITY);
    CHECK(mb_mei_is_registered(TEST_MEI_TYPE) != 0);

    handlers[MB_MEI_MAX_HANDLERS].xor_mask = 0xFFu;
    CHECK(mb_mei_register_handler(TEST_MEI_TYPE, test_handler,
                                  &handlers[MB_MEI_MAX_HANDLERS]) ==
          MB_MEI_OK);
    CHECK(mb_mei_unregister_handler(TEST_MEI_TYPE) == MB_MEI_OK);
    CHECK(mb_mei_unregister_handler(TEST_MEI_TYPE) == MB_MEI_ERROR_NOT_FOUND);
    mb_mei_clear_handlers();
    CHECK(mb_mei_is_registered(0x81u) == 0);

    CHECK(mb_mei_register_handler(TEST_MEI_TYPE, test_handler,
                                  &handlers[0]) == MB_MEI_OK);
    mb_fifo_clear();
    CHECK(mb_mei_is_registered(TEST_MEI_TYPE) != 0);
    mb_init();
    CHECK(mb_mei_is_registered(TEST_MEI_TYPE) == 0);
    return EXIT_SUCCESS;
}

static int test_shared_pdu_and_errors(void)
{
    handler_context_t context = {0u, 0u, 0xFFu, 0u};
    const uint8_t request[] = {MB_MEI_FUNCTION_CODE, TEST_MEI_TYPE,
                               0x11u, 0x22u};
    const uint8_t expected[] = {MB_MEI_FUNCTION_CODE, TEST_MEI_TYPE,
                                0xEEu, 0xDDu};
    const uint8_t empty_request[] = {MB_MEI_FUNCTION_CODE, TEST_MEI_TYPE};
    const uint8_t empty_response[] = {MB_MEI_FUNCTION_CODE, TEST_MEI_TYPE};
    const uint8_t malformed[] = {MB_MEI_FUNCTION_CODE};
    const uint8_t unsupported[] = {MB_MEI_FUNCTION_CODE, 0x81u};
    const uint8_t canopen[] = {
        MB_MEI_FUNCTION_CODE, MB_MEI_TYPE_CANOPEN_GENERAL_REFERENCE
    };
    const uint8_t value_error[] = {0xABu, 0x03u};
    const uint8_t address_error[] = {0xABu, 0x02u};
    const uint8_t server_error[] = {0xABu, 0x04u};

    mb_init();
    CHECK(process_pdu_expect(malformed, sizeof(malformed),
                             MODBUS_PDU_MAX_SIZE, value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(unsupported, sizeof(unsupported),
                             MODBUS_PDU_MAX_SIZE, value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);
    CHECK(process_pdu_expect(canopen, sizeof(canopen),
                             MODBUS_PDU_MAX_SIZE, value_error,
                             sizeof(value_error)) == EXIT_SUCCESS);

    CHECK(mb_mei_register_handler(TEST_MEI_TYPE, test_handler, &context) ==
          MB_MEI_OK);
    CHECK(process_pdu_expect(request, sizeof(request), MODBUS_PDU_MAX_SIZE,
                             expected, sizeof(expected)) == EXIT_SUCCESS);
    CHECK(context.calls == 1u);
    CHECK(process_pdu_expect(empty_request, sizeof(empty_request),
                             MODBUS_PDU_MAX_SIZE, empty_response,
                             sizeof(empty_response)) == EXIT_SUCCESS);
    CHECK(context.calls == 2u);

    context.exception_code = MB_MEI_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    CHECK(process_pdu_expect(request, sizeof(request), MODBUS_PDU_MAX_SIZE,
                             address_error, sizeof(address_error)) ==
          EXIT_SUCCESS);
    context.exception_code = 0xFFu;
    CHECK(process_pdu_expect(request, sizeof(request), MODBUS_PDU_MAX_SIZE,
                             server_error, sizeof(server_error)) ==
          EXIT_SUCCESS);
    context.exception_code = 0u;
    context.force_oversize = 1u;
    CHECK(process_pdu_expect(request, sizeof(request), MODBUS_PDU_MAX_SIZE,
                             server_error, sizeof(server_error)) ==
          EXIT_SUCCESS);
    context.force_oversize = 0u;
    CHECK(process_pdu_expect(request, sizeof(request), 3u,
                             server_error, sizeof(server_error)) ==
          EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static int test_tcp_rtu_and_maximum_pdu(void)
{
    handler_context_t context = {0u, 0u, 0u, 0u};
    uint8_t pdu[MODBUS_PDU_MAX_SIZE];
    uint8_t request[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
    size_t request_length;
    size_t response_length = 0u;

    mb_init();
    CHECK(mb_mei_register_handler(TEST_MEI_TYPE, test_handler, &context) ==
          MB_MEI_OK);
    pdu[0] = MB_MEI_FUNCTION_CODE;
    pdu[1] = TEST_MEI_TYPE;
    for (size_t i = 0u; i < MB_MEI_MAX_DATA_LENGTH; ++i) {
        pdu[2u + i] = (uint8_t)i;
    }

    request_length = make_tcp_adu(pdu, sizeof(pdu), request);
    CHECK(mbtcp_process_adu(request, request_length, response,
                            sizeof(response), &response_length) == 0);
    CHECK(response_length == MODBUS_TCP_ADU_MAX_SIZE);
    CHECK(memcmp(&response[7], pdu, sizeof(pdu)) == 0);

    request_length = make_rtu_adu(1u, pdu, sizeof(pdu), request);
    CHECK(request_length == MODBUS_RTU_ADU_MAX_SIZE);
    response_length = 0u;
    CHECK(mbrtu_process_adu(1u, request, request_length, response,
                            sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response_length == MODBUS_RTU_ADU_MAX_SIZE);
    CHECK(memcmp(&response[1], pdu, sizeof(pdu)) == 0);
    CHECK(mb_crc16(response, response_length) == 0u);
    CHECK(context.calls == 2u);
    return EXIT_SUCCESS;
}

static int test_device_id_regression(void)
{
    static const uint8_t vendor[] = "Vendor";
    static const uint8_t product[] = "Product";
    static const uint8_t revision[] = "1.0";
    const mb_device_id_object_t objects[] = {
        {MB_DEVICE_ID_OBJECT_VENDOR_NAME, vendor, sizeof(vendor) - 1u},
        {MB_DEVICE_ID_OBJECT_PRODUCT_CODE, product, sizeof(product) - 1u},
        {MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION,
         revision, sizeof(revision) - 1u}
    };
    const uint8_t request[] = {
        MB_MEI_FUNCTION_CODE, MB_DEVICE_ID_MEI_TYPE,
        MB_DEVICE_ID_READ_BASIC, 0u
    };
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    mb_init();
    CHECK(mb_device_id_configure(MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL,
                                 objects, 3u) == MB_DEVICE_ID_OK);
    CHECK(mb_process_pdu(request, sizeof(request), response, sizeof(response),
                         &response_length) == 0);
    CHECK(response_length > 7u);
    CHECK(response[0] == MB_MEI_FUNCTION_CODE);
    CHECK(response[1] == MB_DEVICE_ID_MEI_TYPE);
    CHECK(response[2] == MB_DEVICE_ID_READ_BASIC);
    CHECK(response[6] == 3u);
    return EXIT_SUCCESS;
}

static int test_master_builder_parser_and_transaction(void)
{
    const uint8_t request_data[] = {0x10u, 0x20u, 0x30u};
    const uint8_t response_data[] = {0xAAu, 0xBBu};
    const mbrtum_transaction_config_t config = {10u, 1u, 0u, 0u};
    mbrtum_request_t request;
    mbrtum_response_t response;
    mbrtum_mei_response_t mei_response;
    mbrtum_transaction_t transaction;
    transport_t transport;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 99u;
    size_t response_length;

    CHECK(mbrtum_build_mei_request(1u, TEST_MEI_TYPE, request_data,
                                   sizeof(request_data), &request, request_adu,
                                   sizeof(request_adu), &request_length) ==
          MBRTUM_OK);
    CHECK(request_length == 8u);
    CHECK(request_adu[0] == 1u && request_adu[1] == 0x2Bu &&
          request_adu[2] == TEST_MEI_TYPE);
    CHECK(memcmp(&request_adu[3], request_data, sizeof(request_data)) == 0);
    CHECK(mb_crc16(request_adu, request_length) == 0u);
    CHECK(request.function ==
          MBRTUM_FC_ENCAPSULATED_INTERFACE_TRANSPORT);
    CHECK(request.start_address == sizeof(request_data));
    CHECK(request.quantity == 0u && request.value == TEST_MEI_TYPE);

    CHECK(mbrtum_build_mei_request(0u, TEST_MEI_TYPE, NULL, 0u, &request,
                                   request_adu, sizeof(request_adu),
                                   &request_length) ==
          MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(mbrtum_build_mei_request(1u,
                                   MB_MEI_TYPE_CANOPEN_GENERAL_REFERENCE,
                                   NULL, 0u, &request, request_adu,
                                   sizeof(request_adu), &request_length) ==
          MBRTUM_ERROR_FUNCTION);
    CHECK(mbrtum_build_mei_request(1u,
                                   MB_MEI_TYPE_READ_DEVICE_IDENTIFICATION,
                                   NULL, 0u, &request, request_adu,
                                   sizeof(request_adu), &request_length) ==
          MBRTUM_ERROR_FUNCTION);
    CHECK(mbrtum_build_mei_request(1u, TEST_MEI_TYPE, NULL, 1u, &request,
                                   request_adu, sizeof(request_adu),
                                   &request_length) == MBRTUM_ERROR_VALUE);
    CHECK(mbrtum_build_mei_request(1u, TEST_MEI_TYPE, request_data,
                                   sizeof(request_data), &request, request_adu,
                                   7u, &request_length) ==
          MBRTUM_ERROR_CAPACITY);
    CHECK(mbrtum_build_mei_request(1u, TEST_MEI_TYPE, &request_adu[3], 1u,
                                   &request, request_adu, sizeof(request_adu),
                                   &request_length) ==
          MBRTUM_ERROR_ARGUMENT);
    {
        uint8_t maximum_data[MB_MEI_MAX_DATA_LENGTH];

        memset(maximum_data, 0x5Au, sizeof(maximum_data));
        CHECK(mbrtum_build_mei_request(1u, TEST_MEI_TYPE, maximum_data,
                                       sizeof(maximum_data), &request,
                                       request_adu, sizeof(request_adu),
                                       &request_length) == MBRTUM_OK);
        CHECK(request_length == MODBUS_RTU_ADU_MAX_SIZE);
        CHECK(mb_crc16(request_adu, request_length) == 0u);
    }

    CHECK(mbrtum_build_mei_request(1u, TEST_MEI_TYPE, request_data,
                                   sizeof(request_data), &request, request_adu,
                                   sizeof(request_adu), &request_length) ==
          MBRTUM_OK);
    response_adu[0] = 1u;
    response_adu[1] = MBRTUM_FC_ENCAPSULATED_INTERFACE_TRANSPORT;
    response_adu[2] = TEST_MEI_TYPE;
    memcpy(&response_adu[3], response_data, sizeof(response_data));
    response_length = append_crc(response_adu, 5u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_OK);
    CHECK(response.data == &response_adu[2] && response.data_length == 3u);
    CHECK(mbrtum_get_mei_response(&response, &mei_response) == MBRTUM_OK);
    CHECK(mei_response.mei_type == TEST_MEI_TYPE);
    CHECK(mei_response.data == &response_adu[3]);
    CHECK(mei_response.data_length == sizeof(response_data));
    CHECK(memcmp(mei_response.data, response_data, sizeof(response_data)) == 0);

    response_adu[2] = 0x81u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) ==
          MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH);
    response_adu[2] = TEST_MEI_TYPE;
    replace_crc(response_adu, response_length);
    response_adu[response_length - 1u] ^= 1u;
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_ERROR_CRC);

    response_adu[0] = 1u;
    response_adu[1] = 0xABu;
    response_adu[2] = MBRTUM_EXCEPTION_ILLEGAL_DATA_VALUE;
    response_length = append_crc(response_adu, 3u);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response) == MBRTUM_EXCEPTION_RESPONSE);
    CHECK(response.exception_code == MBRTUM_EXCEPTION_ILLEGAL_DATA_VALUE);

    CHECK(mbrtum_build_mei_request(1u, TEST_MEI_TYPE, request_data,
                                   sizeof(request_data), &request, request_adu,
                                   sizeof(request_adu), &request_length) ==
          MBRTUM_OK);
    memset(&transport, 0, sizeof(transport));
    CHECK(mbrtum_transaction_init(&transaction, &config, fake_transmit,
                                  &transport) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request, request_adu,
                                   request_length, 10u) == MBRTUM_TXN_OK);
    CHECK(transport.calls == 1u && transport.length == request_length);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 11u) ==
          MBRTUM_TXN_OK);
    response_adu[0] = 1u;
    response_adu[1] = MBRTUM_FC_ENCAPSULATED_INTERFACE_TRANSPORT;
    response_adu[2] = TEST_MEI_TYPE;
    response_adu[3] = 0x55u;
    response_length = append_crc(response_adu, 4u);
    CHECK(mbrtum_transaction_on_response(&transaction, response_adu,
                                         response_length, 12u) ==
          MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    CHECK(mbrtum_get_mei_response(&transaction.response, &mei_response) ==
          MBRTUM_OK);
    CHECK(mei_response.data_length == 1u && mei_response.data[0] == 0x55u);

    request.value = MB_MEI_TYPE_CANOPEN_GENERAL_REFERENCE;
    CHECK(mbrtum_transaction_init(&transaction, &config, fake_transmit,
                                  &transport) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request, request_adu,
                                   request_length, 20u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_registration_guards() == EXIT_SUCCESS);
    CHECK(test_shared_pdu_and_errors() == EXIT_SUCCESS);
    CHECK(test_tcp_rtu_and_maximum_pdu() == EXIT_SUCCESS);
    CHECK(test_device_id_regression() == EXIT_SUCCESS);
    CHECK(test_master_builder_parser_and_transaction() == EXIT_SUCCESS);
    puts("Modbus FC43 generic MEI transport tests: PASS");
    return EXIT_SUCCESS;
}
