#include "modbus_protocol.h"

#include "modbus_pdu.h"

#include "modbus.h"
#include "modbus_device_id.h"
#include "modbus_device_id_internal.h"
#include "modbus_file_record.h"
#include "modbus_file_record_internal.h"
#include "platform_port.h"

#include <string.h>

#define MB_EX_ILLEGAL_FUNCTION     0x01u
#define MB_EX_ILLEGAL_DATA_ADDRESS 0x02u
#define MB_EX_ILLEGAL_DATA_VALUE   0x03u
#define MB_EX_SERVER_FAILURE       0x04u

static uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static int range_is_valid(uint16_t first, uint16_t quantity, uint32_t limit)
{
    return quantity > 0u && (uint32_t)first < limit &&
           (uint32_t)quantity <= (limit - (uint32_t)first);
}

static uint8_t process_pdu_function(const uint8_t *pdu,
                                    size_t pdu_len,
                                    uint8_t *response,
                                    size_t response_capacity,
                                    size_t *response_len)
{
    uint8_t function;
    uint16_t address;
    uint16_t quantity;
    size_t byte_count;

    if (pdu == NULL || response == NULL || response_len == NULL || pdu_len == 0u) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    function = pdu[0];

    switch (function) {
    case 0x01u: /* Read Coils */
    case 0x02u: /* Read Discrete Inputs */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 2000u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity,
                            function == 0x01u ? MB_MAX_COILS : MB_MAX_DISCRETE_INPUTS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = ((size_t)quantity + 7u) / 8u;
        if (response_capacity < 2u + byte_count) {
            return MB_EX_SERVER_FAILURE;
        }
        response[0] = function;
        response[1] = (uint8_t)byte_count;
        memset(&response[2], 0, byte_count);
        for (uint16_t i = 0; i < quantity; ++i) {
            uint8_t bit = function == 0x01u ? mb_get_coil((uint16_t)(address + i))
                                            : mb_get_dinput((uint16_t)(address + i));
            response[2u + (i / 8u)] |= (uint8_t)((bit & 1u) << (i % 8u));
        }
        *response_len = 2u + byte_count;
        return 0u;

    case 0x03u: /* Read Holding Registers */
    case 0x04u: /* Read Input Registers */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 125u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity,
                            function == 0x03u ? MB_MAX_HREGS : MB_MAX_IREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = (size_t)quantity * 2u;
        if (response_capacity < 2u + byte_count) {
            return MB_EX_SERVER_FAILURE;
        }
        response[0] = function;
        response[1] = (uint8_t)byte_count;
        for (uint16_t i = 0; i < quantity; ++i) {
            uint16_t value = function == 0x03u ? mb_get_hreg((uint16_t)(address + i))
                                               : mb_get_ireg((uint16_t)(address + i));
            write_be16(&response[2u + (2u * i)], value);
        }
        *response_len = 2u + byte_count;
        return 0u;

    case 0x05u: /* Write Single Coil */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        if (!range_is_valid(address, 1u, MB_MAX_COILS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        quantity = read_be16(&pdu[3]);
        if (quantity != 0xFF00u && quantity != 0x0000u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        mb_set_coil(address, quantity == 0xFF00u ? 1u : 0u);
        memcpy(response, pdu, 5u);
        *response_len = 5u;
        return 0u;

    case 0x06u: /* Write Single Holding Register */
        if (pdu_len != 5u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        if (!range_is_valid(address, 1u, MB_MAX_HREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        mb_set_hreg(address, read_be16(&pdu[3]));
        memcpy(response, pdu, 5u);
        *response_len = 5u;
        return 0u;

    case 0x0Fu: /* Write Multiple Coils */
        if (pdu_len < 6u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 1968u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity, MB_MAX_COILS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = ((size_t)quantity + 7u) / 8u;
        if (pdu[5] != byte_count || pdu_len != 6u + byte_count) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        for (uint16_t i = 0; i < quantity; ++i) {
            uint8_t value = (uint8_t)(((uint32_t)pdu[6u + (i / 8u)] >> (i % 8u)) & 1u);
            mb_set_coil((uint16_t)(address + i), value);
        }
        response[0] = function;
        write_be16(&response[1], address);
        write_be16(&response[3], quantity);
        *response_len = 5u;
        return 0u;

    case 0x10u: /* Write Multiple Holding Registers */
        if (pdu_len < 6u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        quantity = read_be16(&pdu[3]);
        if (quantity == 0u || quantity > 123u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(address, quantity, MB_MAX_HREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        byte_count = (size_t)quantity * 2u;
        if (pdu[5] != byte_count || pdu_len != 6u + byte_count) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (response_capacity < 5u) {
            return MB_EX_SERVER_FAILURE;
        }
        for (uint16_t i = 0; i < quantity; ++i) {
            mb_set_hreg((uint16_t)(address + i), read_be16(&pdu[6u + (2u * i)]));
        }
        response[0] = function;
        write_be16(&response[1], address);
        write_be16(&response[3], quantity);
        *response_len = 5u;
        return 0u;

    case MB_FILE_RECORD_READ_FUNCTION_CODE: /* Read File Record */
        return mb_file_record_process_read_request(pdu,
                                                   pdu_len,
                                                   response,
                                                   response_capacity,
                                                   response_len);

    case MB_FILE_RECORD_WRITE_FUNCTION_CODE: /* Write File Record */
        return mb_file_record_process_write_request(pdu,
                                                    pdu_len,
                                                    response,
                                                    response_capacity,
                                                    response_len);

    case 0x16u: { /* Mask Write Holding Register */
        uint16_t and_mask;
        uint16_t or_mask;

        if (pdu_len != 7u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        address = read_be16(&pdu[1]);
        if (!range_is_valid(address, 1u, MB_MAX_HREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        if (response_capacity < 7u) {
            return MB_EX_SERVER_FAILURE;
        }

        and_mask = read_be16(&pdu[3]);
        or_mask = read_be16(&pdu[5]);
        (void)mb_mask_write_hreg(address, and_mask, or_mask);
        memcpy(response, pdu, 7u);
        *response_len = 7u;
        return 0u;
    }

    case 0x17u: { /* Read/Write Multiple Holding Registers */
        uint16_t read_address;
        uint16_t read_quantity;
        uint16_t write_address;
        uint16_t write_quantity;
        size_t write_byte_count;
        size_t read_byte_count;

        if (pdu_len < 10u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        read_address = read_be16(&pdu[1]);
        read_quantity = read_be16(&pdu[3]);
        write_address = read_be16(&pdu[5]);
        write_quantity = read_be16(&pdu[7]);
        if (read_quantity == 0u || read_quantity > 125u ||
            write_quantity == 0u || write_quantity > 121u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!range_is_valid(read_address, read_quantity, MB_MAX_HREGS) ||
            !range_is_valid(write_address, write_quantity, MB_MAX_HREGS)) {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        write_byte_count = (size_t)write_quantity * 2u;
        if ((size_t)pdu[9] != write_byte_count ||
            pdu_len != 10u + write_byte_count) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        read_byte_count = (size_t)read_quantity * 2u;
        if (response_capacity < 2u + read_byte_count) {
            return MB_EX_SERVER_FAILURE;
        }

        for (uint16_t i = 0u; i < write_quantity; ++i) {
            mb_set_hreg((uint16_t)(write_address + i),
                        read_be16(&pdu[10u + ((size_t)i * 2u)]));
        }

        response[0] = function;
        response[1] = (uint8_t)read_byte_count;
        for (uint16_t i = 0u; i < read_quantity; ++i) {
            write_be16(&response[2u + ((size_t)i * 2u)],
                       mb_get_hreg((uint16_t)(read_address + i)));
        }
        *response_len = 2u + read_byte_count;
        return 0u;
    }

    case MB_DEVICE_ID_FUNCTION_CODE: /* Read Device Identification */
        if (pdu_len != 4u) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (pdu[1] != MB_DEVICE_ID_MEI_TYPE) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        return mb_device_id_process_request(pdu[2],
                                            pdu[3],
                                            response,
                                            response_capacity,
                                            response_len);

    default:
        return MB_EX_ILLEGAL_FUNCTION;
    }
}

int mb_process_pdu(const uint8_t *request_pdu,
                   size_t request_pdu_len,
                   uint8_t *response_pdu,
                   size_t response_capacity,
                   size_t *response_pdu_len)
{
    uint8_t exception;

    if (request_pdu == NULL || response_pdu == NULL || response_pdu_len == NULL) {
        return -1;
    }
    *response_pdu_len = 0u;

    if (request_pdu_len == 0u || request_pdu_len > MODBUS_PDU_MAX_SIZE) {
        return -2;
    }
    if (response_capacity < 2u) {
        return -3;
    }

    exception = process_pdu_function(request_pdu,
                                     request_pdu_len,
                                     response_pdu,
                                     response_capacity,
                                     response_pdu_len);
    if (exception != 0u) {
        response_pdu[0] = (uint8_t)(request_pdu[0] | 0x80u);
        response_pdu[1] = exception;
        *response_pdu_len = 2u;
    }

    return 0;
}

int mbtcp_process_adu(const uint8_t *request,
                      size_t request_len,
                      uint8_t *response,
                      size_t response_capacity,
                      size_t *response_len)
{
    uint16_t length_field;
    size_t expected_len;
    size_t response_pdu_len = 0u;
    int pdu_result;

    if (request == NULL || response == NULL || response_len == NULL) {
        return -1;
    }
    *response_len = 0u;

    if (request_len < MBTCP_MBAP_HEADER_SIZE + 1u ||
        request_len > MODBUS_TCP_ADU_MAX_SIZE) {
        return -2;
    }
    if (request[2] != 0u || request[3] != 0u) {
        return -3;
    }

    length_field = read_be16(&request[4]);
    expected_len = 6u + (size_t)length_field;
    if (length_field < 2u || expected_len != request_len) {
        return -4;
    }
    if (response_capacity < MBTCP_MBAP_HEADER_SIZE + 2u) {
        return -5;
    }

    pdu_result = mb_process_pdu(&request[7],
                                request_len - MBTCP_MBAP_HEADER_SIZE,
                                &response[7],
                                response_capacity - MBTCP_MBAP_HEADER_SIZE,
                                &response_pdu_len);
    if (pdu_result < 0) {
        return -6;
    }

    memcpy(response, request, MBTCP_MBAP_HEADER_SIZE);

    length_field = (uint16_t)(1u + response_pdu_len);
    write_be16(&response[4], length_field);
    *response_len = MBTCP_MBAP_HEADER_SIZE + response_pdu_len;
    return 0;
}
