#include "modbus.h"
#include "modbus_crc16.h"
#include "modbus_device_id.h"
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

static const uint8_t vendor[] = "Example Vendor";
static const uint8_t product[] = "STM32-MODBUS";
static const uint8_t revision[] = "1.0";
static const uint8_t url[] = "https://example.invalid";
static const uint8_t product_name[] = "Portable Modbus Core";
static const uint8_t private_value[] = {0xDEu, 0xADu, 0xBEu, 0xEFu};

static const mb_device_id_object_t extended_objects[] = {
    {MB_DEVICE_ID_OBJECT_VENDOR_NAME, vendor, sizeof(vendor) - 1u},
    {MB_DEVICE_ID_OBJECT_PRODUCT_CODE, product, sizeof(product) - 1u},
    {MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION, revision, sizeof(revision) - 1u},
    {MB_DEVICE_ID_OBJECT_VENDOR_URL, url, sizeof(url) - 1u},
    {MB_DEVICE_ID_OBJECT_PRODUCT_NAME, product_name, sizeof(product_name) - 1u},
    {0x80u, private_value, sizeof(private_value)}
};

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

static size_t make_rtu_request(uint8_t address,
                               uint8_t read_code,
                               uint8_t object_id,
                               uint8_t *adu)
{
    adu[0] = address;
    adu[1] = MB_DEVICE_ID_FUNCTION_CODE;
    adu[2] = MB_DEVICE_ID_MEI_TYPE;
    adu[3] = read_code;
    adu[4] = object_id;
    return append_crc(adu, 5u);
}

static size_t make_tcp_request(uint8_t read_code,
                               uint8_t object_id,
                               uint8_t *adu)
{
    adu[0] = 0x12u;
    adu[1] = 0x34u;
    adu[2] = 0u;
    adu[3] = 0u;
    adu[4] = 0u;
    adu[5] = 5u;
    adu[6] = 1u;
    adu[7] = MB_DEVICE_ID_FUNCTION_CODE;
    adu[8] = MB_DEVICE_ID_MEI_TYPE;
    adu[9] = read_code;
    adu[10] = object_id;
    return 11u;
}

static int configure_extended(void)
{
    mb_init();
    return mb_device_id_configure(
        MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL,
        extended_objects,
        sizeof(extended_objects) / sizeof(extended_objects[0]));
}

static int test_configuration_guards(void)
{
    uint8_t too_long[MB_DEVICE_ID_MAX_VALUE_LENGTH + 1u] = {0u};
    uint8_t large_a[200] = {0u};
    uint8_t large_b[200] = {0u};
    uint8_t large_c[200] = {0u};
    const mb_device_id_object_t missing_mandatory[] = {
        {0u, vendor, sizeof(vendor) - 1u},
        {2u, revision, sizeof(revision) - 1u},
        {3u, url, sizeof(url) - 1u}
    };
    const mb_device_id_object_t reserved_object[] = {
        {0u, vendor, sizeof(vendor) - 1u},
        {1u, product, sizeof(product) - 1u},
        {2u, revision, sizeof(revision) - 1u},
        {7u, url, sizeof(url) - 1u}
    };
    const mb_device_id_object_t private_in_regular[] = {
        {0u, vendor, sizeof(vendor) - 1u},
        {1u, product, sizeof(product) - 1u},
        {2u, revision, sizeof(revision) - 1u},
        {0x80u, private_value, sizeof(private_value)}
    };
    const mb_device_id_object_t oversized_object[] = {
        {0u, too_long, sizeof(too_long)},
        {1u, product, sizeof(product) - 1u},
        {2u, revision, sizeof(revision) - 1u}
    };
    const mb_device_id_object_t storage_overflow[] = {
        {0u, large_a, sizeof(large_a)},
        {1u, large_b, sizeof(large_b)},
        {2u, large_c, sizeof(large_c)}
    };

    mb_init();
    CHECK(mb_device_id_is_configured() == 0);
    CHECK(mb_device_id_configure(0u, extended_objects,
                                 sizeof(extended_objects) /
                                     sizeof(extended_objects[0])) ==
          MB_DEVICE_ID_ERROR_CONFORMITY);
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL,
              NULL,
              3u) == MB_DEVICE_ID_ERROR_ARGUMENT);
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL,
              missing_mandatory,
              sizeof(missing_mandatory) / sizeof(missing_mandatory[0])) ==
          MB_DEVICE_ID_ERROR_OBJECTS);
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL,
              reserved_object,
              sizeof(reserved_object) / sizeof(reserved_object[0])) ==
          MB_DEVICE_ID_ERROR_OBJECTS);
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_REGULAR_INDIVIDUAL,
              private_in_regular,
              sizeof(private_in_regular) / sizeof(private_in_regular[0])) ==
          MB_DEVICE_ID_ERROR_OBJECTS);
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL,
              oversized_object,
              sizeof(oversized_object) / sizeof(oversized_object[0])) ==
          MB_DEVICE_ID_ERROR_OBJECTS);
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL,
              storage_overflow,
              sizeof(storage_overflow) / sizeof(storage_overflow[0])) ==
          MB_DEVICE_ID_ERROR_CAPACITY);
    CHECK(configure_extended() == MB_DEVICE_ID_OK);
    CHECK(mb_device_id_is_configured() != 0);
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL,
              reserved_object,
              sizeof(reserved_object) / sizeof(reserved_object[0])) ==
          MB_DEVICE_ID_ERROR_OBJECTS);
    CHECK(mb_device_id_is_configured() != 0);
    mb_device_id_clear();
    CHECK(mb_device_id_is_configured() == 0);
    return EXIT_SUCCESS;
}

static int test_configuration_copies_values(void)
{
    uint8_t mutable_vendor[] = {'V'};
    uint8_t mutable_product[] = {'P'};
    uint8_t mutable_revision[] = {'R'};
    const mb_device_id_object_t objects[] = {
        {0u, mutable_vendor, sizeof(mutable_vendor)},
        {1u, mutable_product, sizeof(mutable_product)},
        {2u, mutable_revision, sizeof(mutable_revision)}
    };
    const uint8_t request[] = {0x2Bu, 0x0Eu, 0x01u, 0x00u};
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    mb_init();
    CHECK(mb_device_id_configure(MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL,
                                 objects,
                                 sizeof(objects) / sizeof(objects[0])) == 0);
    mutable_vendor[0] = 'X';
    mutable_product[0] = 'Y';
    mutable_revision[0] = 'Z';

    CHECK(mb_process_pdu(request, sizeof(request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response_length == 16u);
    CHECK(response[9] == 'V' && response[12] == 'P' && response[15] == 'R');
    return EXIT_SUCCESS;
}

static int test_basic_regular_extended_and_specific(void)
{
    const uint8_t basic_request[] = {0x2Bu, 0x0Eu, 0x01u, 0x00u};
    const uint8_t regular_request[] = {0x2Bu, 0x0Eu, 0x02u, 0x00u};
    const uint8_t extended_request[] = {0x2Bu, 0x0Eu, 0x03u, 0x00u};
    const uint8_t specific_request[] = {0x2Bu, 0x0Eu, 0x04u, 0x80u};
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    CHECK(configure_extended() == MB_DEVICE_ID_OK);

    CHECK(mb_process_pdu(basic_request, sizeof(basic_request),
                         response, sizeof(response), &response_length) == 0);
    CHECK(response[0] == 0x2Bu && response[1] == 0x0Eu);
    CHECK(response[2] == 0x01u && response[3] == 0x83u);
    CHECK(response[4] == 0u && response[5] == 0u && response[6] == 3u);
    CHECK(response[7] == 0u);

    CHECK(mb_process_pdu(regular_request, sizeof(regular_request),
                         response, sizeof(response), &response_length) == 0);
    CHECK(response[6] == 5u);
    CHECK(response[response_length - (sizeof(product_name) - 1u) - 2u] == 4u);

    CHECK(mb_process_pdu(extended_request, sizeof(extended_request),
                         response, sizeof(response), &response_length) == 0);
    CHECK(response[6] == 6u);
    CHECK(response[response_length - sizeof(private_value) - 2u] == 0x80u);

    CHECK(mb_process_pdu(specific_request, sizeof(specific_request),
                         response, sizeof(response), &response_length) == 0);
    CHECK(response[2] == 4u && response[4] == 0u && response[5] == 0u);
    CHECK(response[6] == 1u && response[7] == 0x80u);
    CHECK(response[8] == sizeof(private_value));
    CHECK(memcmp(&response[9], private_value, sizeof(private_value)) == 0);
    return EXIT_SUCCESS;
}

static int test_segmentation_restart_and_capacity(void)
{
    const uint8_t a[] = "AAAAAAAAAA";
    const uint8_t b[] = "BBBBBBBBBB";
    const uint8_t c[] = "CCCCCCCCCC";
    const mb_device_id_object_t objects[] = {
        {0u, a, sizeof(a) - 1u},
        {1u, b, sizeof(b) - 1u},
        {2u, c, sizeof(c) - 1u}
    };
    const uint8_t first_request[] = {0x2Bu, 0x0Eu, 0x01u, 0x00u};
    const uint8_t second_request[] = {0x2Bu, 0x0Eu, 0x01u, 0x02u};
    const uint8_t unknown_request[] = {0x2Bu, 0x0Eu, 0x01u, 0x55u};
    uint8_t response[31];
    size_t response_length = 0u;

    mb_init();
    CHECK(mb_device_id_configure(MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL,
                                 objects,
                                 sizeof(objects) / sizeof(objects[0])) == 0);
    CHECK(mb_process_pdu(first_request, sizeof(first_request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response_length == sizeof(response));
    CHECK(response[4] == 0xFFu && response[5] == 2u && response[6] == 2u);

    CHECK(mb_process_pdu(second_request, sizeof(second_request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[4] == 0u && response[5] == 0u && response[6] == 1u);
    CHECK(response[7] == 2u);

    CHECK(mb_process_pdu(unknown_request, sizeof(unknown_request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[7] == 0u);

    CHECK(mb_process_pdu(first_request, sizeof(first_request), response,
                         8u, &response_length) == 0);
    CHECK(response_length == 2u && response[0] == 0xABu && response[1] == 0x04u);
    return EXIT_SUCCESS;
}

static int test_request_validation_and_conformity_fallback(void)
{
    const mb_device_id_object_t basic_objects[] = {
        {0u, vendor, sizeof(vendor) - 1u},
        {1u, product, sizeof(product) - 1u},
        {2u, revision, sizeof(revision) - 1u}
    };
    const uint8_t short_request[] = {0x2Bu, 0x0Eu, 0x01u};
    const uint8_t wrong_mei[] = {0x2Bu, 0x0Du, 0x01u, 0u};
    const uint8_t bad_code[] = {0x2Bu, 0x0Eu, 0x05u, 0u};
    const uint8_t unknown_specific[] = {0x2Bu, 0x0Eu, 0x04u, 0x55u};
    const uint8_t extended_request[] = {0x2Bu, 0x0Eu, 0x03u, 0u};
    uint8_t response[MODBUS_PDU_MAX_SIZE];
    size_t response_length = 0u;

    mb_init();
    CHECK(mb_process_pdu(extended_request, sizeof(extended_request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[0] == 0xABu && response[1] == 0x04u);

    CHECK(mb_device_id_configure(MB_DEVICE_ID_CONFORMITY_BASIC_STREAM,
                                 basic_objects,
                                 sizeof(basic_objects) /
                                     sizeof(basic_objects[0])) == 0);
    CHECK(mb_process_pdu(short_request, sizeof(short_request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[0] == 0xABu && response[1] == 0x03u);
    CHECK(mb_process_pdu(wrong_mei, sizeof(wrong_mei), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[0] == 0xABu && response[1] == 0x03u);
    CHECK(mb_process_pdu(bad_code, sizeof(bad_code), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[0] == 0xABu && response[1] == 0x03u);
    CHECK(mb_process_pdu(unknown_specific, sizeof(unknown_specific), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[0] == 0xABu && response[1] == 0x03u);

    CHECK(mb_process_pdu(extended_request, sizeof(extended_request), response,
                         sizeof(response), &response_length) == 0);
    CHECK(response[2] == 3u && response[3] == 1u && response[6] == 3u);
    return EXIT_SUCCESS;
}

static int test_tcp_rtu_and_maximum_pdu(void)
{
    uint8_t request[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t large_value[MB_DEVICE_ID_MAX_VALUE_LENGTH];
    mb_device_id_object_t max_objects[] = {
        {0u, NULL, 0u},
        {1u, NULL, 0u},
        {2u, NULL, 0u},
        {0x80u, large_value, sizeof(large_value)}
    };
    size_t request_length;
    size_t response_length = 0u;

    CHECK(configure_extended() == 0);
    request_length = make_tcp_request(1u, 0u, request);
    CHECK(mbtcp_process_adu(request, request_length, response,
                            sizeof(response), &response_length) == 0);
    CHECK(response[7] == 0x2Bu && response[8] == 0x0Eu);

    request_length = make_rtu_request(1u, 1u, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length, response,
                            sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response[0] == 1u && response[1] == 0x2Bu);
    CHECK(mb_crc16(response, response_length) == 0u);

    request_length = make_rtu_request(0u, 1u, 0u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length, response,
                            sizeof(response), &response_length) ==
          MBRTU_NO_RESPONSE);
    CHECK(response_length == 0u);

    memset(large_value, 0xA5, sizeof(large_value));
    mb_init();
    CHECK(mb_device_id_configure(
              MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL,
              max_objects,
              sizeof(max_objects) / sizeof(max_objects[0])) == 0);
    request_length = make_rtu_request(1u, 4u, 0x80u, request);
    CHECK(mbrtu_process_adu(1u, request, request_length, response,
                            sizeof(response), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(response_length == MODBUS_RTU_ADU_MAX_SIZE);
    CHECK(response[1] == 0x2Bu && response[8] == 0x80u &&
          response[9] == MB_DEVICE_ID_MAX_VALUE_LENGTH);
    CHECK(mb_crc16(response, response_length) == 0u);
    return EXIT_SUCCESS;
}

typedef struct {
    unsigned calls;
    uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t length;
} fake_transport_t;

static int fake_transmit(void *context, const uint8_t *adu, size_t length)
{
    fake_transport_t *transport = (fake_transport_t *)context;

    ++transport->calls;
    memcpy(transport->adu, adu, length);
    transport->length = length;
    return MBRTUM_TXN_TRANSMIT_ACCEPTED;
}

static int test_master_builder_parser_and_transaction(void)
{
    mbrtum_request_t request;
    mbrtum_response_t response_view;
    mbrtum_device_id_response_t device_view;
    mbrtum_device_id_object_t object;
    mbrtum_transaction_t transaction;
    mbrtum_transaction_config_t config = {100u, 0u, 0u, 0u};
    fake_transport_t transport = {0u, {0u}, 0u};
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    mbrtum_request_t specific_request;
    uint8_t specific_request_adu[7u];
    size_t specific_request_length = 0u;
    size_t request_length = 0u;
    size_t response_length = 0u;

    CHECK(configure_extended() == 0);
    CHECK(mbrtum_build_read_device_identification_request(
              1u, MB_DEVICE_ID_READ_EXTENDED, 0u, &request,
              request_adu, sizeof(request_adu), &request_length) == MBRTUM_OK);
    CHECK(request_length == 7u && request_adu[1] == 0x2Bu &&
          request_adu[2] == 0x0Eu && mb_crc16(request_adu, request_length) == 0u);
    CHECK(request.start_address == MB_DEVICE_ID_READ_EXTENDED &&
          request.quantity == 0u && request.value == MB_DEVICE_ID_MEI_TYPE);

    CHECK(mbrtu_process_adu(1u, request_adu, request_length, response_adu,
                            sizeof(response_adu), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response_view) == MBRTUM_OK);
    CHECK(mbrtum_get_device_id_response(&response_view, &device_view) ==
          MBRTUM_OK);
    CHECK(device_view.object_count == 6u &&
          device_view.conformity_level == 0x83u);
    CHECK(mbrtum_get_device_id_object(&device_view, 0u, &object) == MBRTUM_OK);
    CHECK(object.object_id == 0u && object.value_length == sizeof(vendor) - 1u);
    CHECK(memcmp(object.value, vendor, object.value_length) == 0);
    CHECK(mbrtum_get_device_id_object(&device_view, 5u, &object) == MBRTUM_OK);
    CHECK(object.object_id == 0x80u);
    CHECK(mbrtum_get_device_id_object(&device_view, 6u, &object) ==
          MBRTUM_ERROR_INDEX);

    response_adu[2] = 0x0Du;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response_view) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);

    CHECK(mbrtu_process_adu(1u, request_adu, request_length, response_adu,
                            sizeof(response_adu), &response_length) ==
          MBRTU_RESPONSE_READY);
    response_adu[5] = 0xFFu;
    response_adu[6] = 0u;
    replace_crc(response_adu, response_length);
    CHECK(mbrtum_process_response(&request, response_adu, response_length,
                                  &response_view) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);

    CHECK(mbrtum_transaction_init(&transaction, &config, fake_transmit,
                                  &transport) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request, request_adu,
                                   request_length, 10u) == MBRTUM_TXN_OK);
    CHECK(transport.calls == 1u && transport.length == request_length);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 11u) ==
          MBRTUM_TXN_OK);
    CHECK(mbrtu_process_adu(1u, request_adu, request_length, response_adu,
                            sizeof(response_adu), &response_length) ==
          MBRTU_RESPONSE_READY);
    CHECK(mbrtum_transaction_on_response(&transaction, response_adu,
                                         response_length, 12u) ==
          MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE &&
          transaction.result == MBRTUM_TXN_RESULT_SUCCESS);

    CHECK(mbrtum_build_read_device_identification_request(
              1u, MB_DEVICE_ID_READ_SPECIFIC, 0x80u, &specific_request,
              specific_request_adu, sizeof(specific_request_adu),
              &specific_request_length) == MBRTUM_OK);
    CHECK(specific_request_length == sizeof(specific_request_adu));
    response_adu[0] = 1u;
    response_adu[1] = MBRTUM_FC_READ_DEVICE_IDENTIFICATION;
    response_adu[2] = MB_DEVICE_ID_MEI_TYPE;
    response_adu[3] = MB_DEVICE_ID_READ_SPECIFIC;
    response_adu[4] = MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL;
    response_adu[5] = 0u;
    response_adu[6] = 0u;
    response_adu[7] = 1u;
    response_adu[8] = 0x80u;
    response_adu[9] = 1u;
    response_adu[10] = 0xAAu;
    response_length = append_crc(response_adu, 11u);
    CHECK(mbrtum_process_response(&specific_request,
                                  response_adu,
                                  response_length,
                                  &response_view) ==
          MBRTUM_ERROR_MALFORMED_RESPONSE);

    request.quantity = 0x100u;
    CHECK(mbrtum_transaction_init(&transaction, &config, fake_transmit,
                                  &transport) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction, &request, request_adu,
                                   request_length, 20u) ==
          MBRTUM_TXN_ERROR_REQUEST);
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_configuration_guards() == EXIT_SUCCESS);
    CHECK(test_configuration_copies_values() == EXIT_SUCCESS);
    CHECK(test_basic_regular_extended_and_specific() == EXIT_SUCCESS);
    CHECK(test_segmentation_restart_and_capacity() == EXIT_SUCCESS);
    CHECK(test_request_validation_and_conformity_fallback() == EXIT_SUCCESS);
    CHECK(test_tcp_rtu_and_maximum_pdu() == EXIT_SUCCESS);
    CHECK(test_master_builder_parser_and_transaction() == EXIT_SUCCESS);
    puts("Modbus FC43/14 device identification tests: PASS");
    return EXIT_SUCCESS;
}
