#include "modbus_rtu_master.h"

#include "modbus_crc16.h"

#include <string.h>

#define MBRTUM_FIXED_REQUEST_ADU_SIZE 8u
#define MBRTUM_WRITE_ACK_ADU_SIZE 8u
#define MBRTUM_EXCEPTION_ADU_SIZE 5u

static uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8u) | (uint16_t)p[1]);
}

static void write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8u);
    p[1] = (uint8_t)value;
}

static void clear_request(mbrtum_request_t *request)
{
    if (request != NULL) {
        memset(request, 0, sizeof(*request));
    }
}

static void clear_response(mbrtum_response_t *response)
{
    if (response != NULL) {
        memset(response, 0, sizeof(*response));
    }
}

static int unicast_address_is_valid(uint8_t slave_address)
{
    return slave_address >= MODBUS_RTU_ADDRESS_MIN &&
           slave_address <= MODBUS_RTU_ADDRESS_MAX;
}

static int write_address_is_valid(uint8_t slave_address)
{
    return slave_address == MODBUS_RTU_BROADCAST_ADDRESS ||
           unicast_address_is_valid(slave_address);
}

static int address_range_is_valid(uint16_t start_address, uint16_t quantity)
{
    return quantity > 0u &&
           (uint32_t)start_address + (uint32_t)quantity <= 65536u;
}


static int device_id_code_is_valid(uint16_t read_device_id_code)
{
    return read_device_id_code >= MB_DEVICE_ID_READ_BASIC &&
           read_device_id_code <= MB_DEVICE_ID_READ_SPECIFIC;
}

static int device_id_conformity_is_valid(uint8_t conformity_level)
{
    return conformity_level == MB_DEVICE_ID_CONFORMITY_BASIC_STREAM ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_REGULAR_STREAM ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_EXTENDED_STREAM ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_REGULAR_INDIVIDUAL ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL;
}

static uint8_t device_id_conformity_category(uint8_t conformity_level)
{
    return (uint8_t)(conformity_level & 0x7Fu);
}

static int device_id_object_matches_category(uint8_t object_id,
                                             uint8_t category)
{
    if (category == MB_DEVICE_ID_READ_BASIC) {
        return object_id <= MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION;
    }
    if (category == MB_DEVICE_ID_READ_REGULAR) {
        return object_id <= MB_DEVICE_ID_OBJECT_USER_APPLICATION_NAME;
    }
    return category == MB_DEVICE_ID_READ_EXTENDED &&
           (object_id <= MB_DEVICE_ID_OBJECT_USER_APPLICATION_NAME ||
            object_id >= MB_DEVICE_ID_PRIVATE_OBJECT_MIN);
}

static int packed_bit_padding_is_zero(const uint8_t *data,
                                      size_t data_length,
                                      uint16_t quantity)
{
    uint8_t remainder = (uint8_t)(quantity % 8u);

    if (remainder == 0u) {
        return 1;
    }
    if (data == NULL || data_length == 0u) {
        return 0;
    }

    {
        uint8_t valid_mask = (uint8_t)((1u << remainder) - 1u);
        uint8_t padding_mask = (uint8_t)~valid_mask;

        return (data[data_length - 1u] & padding_mask) == 0u;
    }
}

static void finish_request(mbrtum_request_t *request,
                           uint8_t slave_address,
                           uint8_t function,
                           uint16_t start_address,
                           uint16_t quantity,
                           uint16_t value)
{
    request->slave_address = slave_address;
    request->function = function;
    request->start_address = start_address;
    request->quantity = quantity;
    request->value = value;
    request->expects_response = (uint8_t)(
        slave_address == MODBUS_RTU_BROADCAST_ADDRESS ? 0u : 1u);
}

static size_t append_crc(uint8_t *adu, size_t length_without_crc)
{
    uint16_t crc = mb_crc16(adu, length_without_crc);

    adu[length_without_crc] = (uint8_t)crc;
    adu[length_without_crc + 1u] = (uint8_t)(crc >> 8u);
    return length_without_crc + MODBUS_RTU_CRC_SIZE;
}

static int build_fixed_request(uint8_t slave_address,
                               uint8_t function,
                               uint16_t first,
                               uint16_t second,
                               mbrtum_request_t *request,
                               uint16_t request_quantity,
                               uint16_t request_value,
                               uint8_t *request_adu,
                               size_t request_adu_capacity,
                               size_t *request_adu_length)
{
    if (request == NULL || request_adu == NULL || request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    *request_adu_length = 0u;
    clear_request(request);

    if (request_adu_capacity < MBRTUM_FIXED_REQUEST_ADU_SIZE) {
        return MBRTUM_ERROR_CAPACITY;
    }

    request_adu[0] = slave_address;
    request_adu[1] = function;
    write_be16(&request_adu[2], first);
    write_be16(&request_adu[4], second);
    *request_adu_length = append_crc(request_adu, 6u);
    finish_request(request,
                   slave_address,
                   function,
                   first,
                   request_quantity,
                   request_value);
    return MBRTUM_OK;
}

int mbrtum_build_read_bits_request(uint8_t slave_address,
                                   uint8_t function,
                                   uint16_t start_address,
                                   uint16_t quantity,
                                   mbrtum_request_t *request,
                                   uint8_t *request_adu,
                                   size_t request_adu_capacity,
                                   size_t *request_adu_length)
{
    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (request == NULL || request_adu == NULL || request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!unicast_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (function != MBRTUM_FC_READ_COILS &&
        function != MBRTUM_FC_READ_DISCRETE_INPUTS) {
        return MBRTUM_ERROR_FUNCTION;
    }
    if (quantity == 0u || quantity > 2000u ||
        !address_range_is_valid(start_address, quantity)) {
        return MBRTUM_ERROR_QUANTITY;
    }

    return build_fixed_request(slave_address,
                               function,
                               start_address,
                               quantity,
                               request,
                               quantity,
                               0u,
                               request_adu,
                               request_adu_capacity,
                               request_adu_length);
}

int mbrtum_build_read_registers_request(uint8_t slave_address,
                                        uint8_t function,
                                        uint16_t start_address,
                                        uint16_t quantity,
                                        mbrtum_request_t *request,
                                        uint8_t *request_adu,
                                        size_t request_adu_capacity,
                                        size_t *request_adu_length)
{
    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (request == NULL || request_adu == NULL || request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!unicast_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (function != MBRTUM_FC_READ_HOLDING_REGISTERS &&
        function != MBRTUM_FC_READ_INPUT_REGISTERS) {
        return MBRTUM_ERROR_FUNCTION;
    }
    if (quantity == 0u || quantity > 125u ||
        !address_range_is_valid(start_address, quantity)) {
        return MBRTUM_ERROR_QUANTITY;
    }

    return build_fixed_request(slave_address,
                               function,
                               start_address,
                               quantity,
                               request,
                               quantity,
                               0u,
                               request_adu,
                               request_adu_capacity,
                               request_adu_length);
}

int mbrtum_build_write_single_coil_request(uint8_t slave_address,
                                           uint16_t address,
                                           uint8_t value,
                                           mbrtum_request_t *request,
                                           uint8_t *request_adu,
                                           size_t request_adu_capacity,
                                           size_t *request_adu_length)
{
    uint16_t encoded_value;

    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (request == NULL || request_adu == NULL || request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!write_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (value > 1u) {
        return MBRTUM_ERROR_VALUE;
    }

    encoded_value = value == 0u ? 0x0000u : 0xFF00u;
    return build_fixed_request(slave_address,
                               MBRTUM_FC_WRITE_SINGLE_COIL,
                               address,
                               encoded_value,
                               request,
                               1u,
                               encoded_value,
                               request_adu,
                               request_adu_capacity,
                               request_adu_length);
}

int mbrtum_build_write_single_register_request(uint8_t slave_address,
                                               uint16_t address,
                                               uint16_t value,
                                               mbrtum_request_t *request,
                                               uint8_t *request_adu,
                                               size_t request_adu_capacity,
                                               size_t *request_adu_length)
{
    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (request == NULL || request_adu == NULL || request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!write_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }

    return build_fixed_request(slave_address,
                               MBRTUM_FC_WRITE_SINGLE_REGISTER,
                               address,
                               value,
                               request,
                               1u,
                               value,
                               request_adu,
                               request_adu_capacity,
                               request_adu_length);
}

int mbrtum_build_write_multiple_coils_request(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *packed_values,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    size_t byte_count;
    size_t length_without_crc;
    uint8_t remainder;

    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (packed_values == NULL || request == NULL || request_adu == NULL ||
        request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!write_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (quantity == 0u || quantity > 1968u ||
        !address_range_is_valid(start_address, quantity)) {
        return MBRTUM_ERROR_QUANTITY;
    }

    byte_count = ((size_t)quantity + 7u) / 8u;
    length_without_crc = 7u + byte_count;
    if (request_adu_capacity < length_without_crc + MODBUS_RTU_CRC_SIZE) {
        return MBRTUM_ERROR_CAPACITY;
    }

    request_adu[0] = slave_address;
    request_adu[1] = MBRTUM_FC_WRITE_MULTIPLE_COILS;
    write_be16(&request_adu[2], start_address);
    write_be16(&request_adu[4], quantity);
    request_adu[6] = (uint8_t)byte_count;
    memcpy(&request_adu[7], packed_values, byte_count);

    remainder = (uint8_t)(quantity % 8u);
    if (remainder != 0u) {
        uint8_t mask = (uint8_t)((1u << remainder) - 1u);
        request_adu[7u + byte_count - 1u] &= mask;
    }

    *request_adu_length = append_crc(request_adu, length_without_crc);
    finish_request(request,
                   slave_address,
                   MBRTUM_FC_WRITE_MULTIPLE_COILS,
                   start_address,
                   quantity,
                   0u);
    return MBRTUM_OK;
}

int mbrtum_build_write_multiple_registers_request(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *values,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    size_t byte_count;
    size_t length_without_crc;

    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (values == NULL || request == NULL || request_adu == NULL ||
        request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!write_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (quantity == 0u || quantity > 123u ||
        !address_range_is_valid(start_address, quantity)) {
        return MBRTUM_ERROR_QUANTITY;
    }

    byte_count = (size_t)quantity * 2u;
    length_without_crc = 7u + byte_count;
    if (request_adu_capacity < length_without_crc + MODBUS_RTU_CRC_SIZE) {
        return MBRTUM_ERROR_CAPACITY;
    }

    request_adu[0] = slave_address;
    request_adu[1] = MBRTUM_FC_WRITE_MULTIPLE_REGISTERS;
    write_be16(&request_adu[2], start_address);
    write_be16(&request_adu[4], quantity);
    request_adu[6] = (uint8_t)byte_count;
    for (uint16_t i = 0u; i < quantity; ++i) {
        write_be16(&request_adu[7u + ((size_t)i * 2u)], values[i]);
    }

    *request_adu_length = append_crc(request_adu, length_without_crc);
    finish_request(request,
                   slave_address,
                   MBRTUM_FC_WRITE_MULTIPLE_REGISTERS,
                   start_address,
                   quantity,
                   0u);
    return MBRTUM_OK;
}

int mbrtum_build_read_write_multiple_registers_request(
    uint8_t slave_address,
    uint16_t read_start_address,
    uint16_t read_quantity,
    uint16_t write_start_address,
    uint16_t write_quantity,
    const uint16_t *write_values,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    size_t byte_count;
    size_t length_without_crc;

    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (write_values == NULL || request == NULL || request_adu == NULL ||
        request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!unicast_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (read_quantity == 0u || read_quantity > 125u ||
        !address_range_is_valid(read_start_address, read_quantity) ||
        write_quantity == 0u || write_quantity > 121u ||
        !address_range_is_valid(write_start_address, write_quantity)) {
        return MBRTUM_ERROR_QUANTITY;
    }

    byte_count = (size_t)write_quantity * 2u;
    length_without_crc = 11u + byte_count;
    if (request_adu_capacity < length_without_crc + MODBUS_RTU_CRC_SIZE) {
        return MBRTUM_ERROR_CAPACITY;
    }

    request_adu[0] = slave_address;
    request_adu[1] = MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS;
    write_be16(&request_adu[2], read_start_address);
    write_be16(&request_adu[4], read_quantity);
    write_be16(&request_adu[6], write_start_address);
    write_be16(&request_adu[8], write_quantity);
    request_adu[10] = (uint8_t)byte_count;
    for (uint16_t i = 0u; i < write_quantity; ++i) {
        write_be16(&request_adu[11u + ((size_t)i * 2u)], write_values[i]);
    }

    *request_adu_length = append_crc(request_adu, length_without_crc);
    finish_request(request,
                   slave_address,
                   MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS,
                   read_start_address,
                   read_quantity,
                   0u);
    request->write_start_address = write_start_address;
    request->write_quantity = write_quantity;
    return MBRTUM_OK;
}

int mbrtum_build_read_device_identification_request(
    uint8_t slave_address,
    uint8_t read_device_id_code,
    uint8_t object_id,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);

    if (request == NULL || request_adu == NULL || request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!unicast_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (!device_id_code_is_valid(read_device_id_code)) {
        return MBRTUM_ERROR_VALUE;
    }
    if (request_adu_capacity < 7u) {
        return MBRTUM_ERROR_CAPACITY;
    }

    request_adu[0] = slave_address;
    request_adu[1] = MBRTUM_FC_READ_DEVICE_IDENTIFICATION;
    request_adu[2] = MB_DEVICE_ID_MEI_TYPE;
    request_adu[3] = read_device_id_code;
    request_adu[4] = object_id;
    *request_adu_length = append_crc(request_adu, 5u);
    finish_request(request,
                   slave_address,
                   MBRTUM_FC_READ_DEVICE_IDENTIFICATION,
                   read_device_id_code,
                   object_id,
                   MB_DEVICE_ID_MEI_TYPE);
    return MBRTUM_OK;
}

static int diagnostic_subfunction_is_state_changing(uint16_t subfunction)
{
    return subfunction == MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS ||
           subfunction == MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY ||
           subfunction == MBRTU_DIAG_SUB_CLEAR_COUNTERS_AND_REGISTER ||
           subfunction == MBRTU_DIAG_SUB_CLEAR_OVERRUN_COUNTER;
}

static int diagnostic_subfunction_is_supported(uint16_t subfunction)
{
    switch (subfunction) {
    case MBRTU_DIAG_SUB_RETURN_QUERY_DATA:
    case MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS:
    case MBRTU_DIAG_SUB_RETURN_DIAGNOSTIC_REGISTER:
    case MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY:
    case MBRTU_DIAG_SUB_CLEAR_COUNTERS_AND_REGISTER:
    case MBRTU_DIAG_SUB_RETURN_BUS_MESSAGE_COUNT:
    case MBRTU_DIAG_SUB_RETURN_BUS_COMM_ERROR_COUNT:
    case MBRTU_DIAG_SUB_RETURN_BUS_EXCEPTION_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_MESSAGE_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_NO_RESPONSE_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_NAK_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_BUSY_COUNT:
    case MBRTU_DIAG_SUB_RETURN_BUS_OVERRUN_COUNT:
    case MBRTU_DIAG_SUB_CLEAR_OVERRUN_COUNTER:
        return 1;
    default:
        return 0;
    }
}

static int build_function_only_request(uint8_t slave_address,
                                       uint8_t function,
                                       mbrtum_request_t *request,
                                       uint8_t *request_adu,
                                       size_t request_adu_capacity,
                                       size_t *request_adu_length)
{
    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);
    if (request == NULL || request_adu == NULL ||
        request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!unicast_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }
    if (request_adu_capacity < MODBUS_RTU_ADU_MIN_SIZE) {
        return MBRTUM_ERROR_CAPACITY;
    }

    request_adu[0] = slave_address;
    request_adu[1] = function;
    *request_adu_length = append_crc(request_adu, 2u);
    finish_request(request, slave_address, function, 0u, 0u, 0u);
    return MBRTUM_OK;
}

int mbrtum_build_read_exception_status_request(
    uint8_t slave_address,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    return build_function_only_request(slave_address,
                                       MBRTUM_FC_READ_EXCEPTION_STATUS,
                                       request,
                                       request_adu,
                                       request_adu_capacity,
                                       request_adu_length);
}

int mbrtum_build_get_comm_event_counter_request(
    uint8_t slave_address,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    return build_function_only_request(slave_address,
                                       MBRTUM_FC_GET_COMM_EVENT_COUNTER,
                                       request,
                                       request_adu,
                                       request_adu_capacity,
                                       request_adu_length);
}

int mbrtum_build_get_comm_event_log_request(
    uint8_t slave_address,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    return build_function_only_request(slave_address,
                                       MBRTUM_FC_GET_COMM_EVENT_LOG,
                                       request,
                                       request_adu,
                                       request_adu_capacity,
                                       request_adu_length);
}

int mbrtum_build_diagnostics_request(
    uint8_t slave_address,
    uint16_t subfunction,
    const uint8_t *diagnostic_data,
    size_t diagnostic_data_length,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length)
{
    size_t length_without_crc;
    uint16_t data_word = 0u;

    if (request_adu_length != NULL) {
        *request_adu_length = 0u;
    }
    clear_request(request);
    if (request == NULL || request_adu == NULL ||
        request_adu_length == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!diagnostic_subfunction_is_supported(subfunction)) {
        return MBRTUM_ERROR_FUNCTION;
    }
    if (diagnostic_data_length > 250u ||
        (diagnostic_data_length % 2u) != 0u ||
        (diagnostic_data_length != 0u && diagnostic_data == NULL)) {
        return MBRTUM_ERROR_VALUE;
    }
    if (slave_address == MODBUS_RTU_BROADCAST_ADDRESS) {
        if (!diagnostic_subfunction_is_state_changing(subfunction)) {
            return MBRTUM_ERROR_SLAVE_ADDRESS;
        }
    } else if (!unicast_address_is_valid(slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }

    if (subfunction != MBRTU_DIAG_SUB_RETURN_QUERY_DATA) {
        if (diagnostic_data_length != 2u) {
            return MBRTUM_ERROR_VALUE;
        }
        data_word = read_be16(diagnostic_data);
        if (subfunction == MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS) {
            if (data_word != 0x0000u && data_word != 0xFF00u) {
                return MBRTUM_ERROR_VALUE;
            }
        } else if (data_word != 0u) {
            return MBRTUM_ERROR_VALUE;
        }
    }

    length_without_crc = 4u + diagnostic_data_length;
    if (request_adu_capacity <
        length_without_crc + MODBUS_RTU_CRC_SIZE) {
        return MBRTUM_ERROR_CAPACITY;
    }

    request_adu[0] = slave_address;
    request_adu[1] = MBRTUM_FC_DIAGNOSTICS;
    write_be16(&request_adu[2], subfunction);
    if (diagnostic_data_length != 0u) {
        memcpy(&request_adu[4], diagnostic_data, diagnostic_data_length);
    }
    *request_adu_length = append_crc(request_adu, length_without_crc);
    finish_request(request,
                   slave_address,
                   MBRTUM_FC_DIAGNOSTICS,
                   subfunction,
                   (uint16_t)diagnostic_data_length,
                   0u);
    request->expects_response = (uint8_t)(
        slave_address != MODBUS_RTU_BROADCAST_ADDRESS &&
        subfunction != MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY);
    return MBRTUM_OK;
}

static int validate_request_descriptor(const mbrtum_request_t *request)
{
    if (request->expects_response == 0u) {
        return MBRTUM_ERROR_RESPONSE_NOT_EXPECTED;
    }
    if (request->expects_response != 1u) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (!unicast_address_is_valid(request->slave_address)) {
        return MBRTUM_ERROR_SLAVE_ADDRESS;
    }

    switch (request->function) {
    case MBRTUM_FC_READ_COILS:
    case MBRTUM_FC_READ_DISCRETE_INPUTS:
        if (request->quantity == 0u || request->quantity > 2000u ||
            !address_range_is_valid(request->start_address, request->quantity)) {
            return MBRTUM_ERROR_QUANTITY;
        }
        break;
    case MBRTUM_FC_READ_HOLDING_REGISTERS:
    case MBRTUM_FC_READ_INPUT_REGISTERS:
        if (request->quantity == 0u || request->quantity > 125u ||
            !address_range_is_valid(request->start_address, request->quantity)) {
            return MBRTUM_ERROR_QUANTITY;
        }
        break;
    case MBRTUM_FC_WRITE_SINGLE_COIL:
        if (request->quantity != 1u ||
            (request->value != 0x0000u && request->value != 0xFF00u)) {
            return MBRTUM_ERROR_VALUE;
        }
        break;
    case MBRTUM_FC_WRITE_SINGLE_REGISTER:
        if (request->quantity != 1u) {
            return MBRTUM_ERROR_QUANTITY;
        }
        break;
    case MBRTUM_FC_WRITE_MULTIPLE_COILS:
        if (request->quantity == 0u || request->quantity > 1968u ||
            !address_range_is_valid(request->start_address, request->quantity)) {
            return MBRTUM_ERROR_QUANTITY;
        }
        break;
    case MBRTUM_FC_WRITE_MULTIPLE_REGISTERS:
        if (request->quantity == 0u || request->quantity > 123u ||
            !address_range_is_valid(request->start_address, request->quantity)) {
            return MBRTUM_ERROR_QUANTITY;
        }
        break;
    case MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS:
        if (request->quantity == 0u || request->quantity > 125u ||
            !address_range_is_valid(request->start_address, request->quantity) ||
            request->write_quantity == 0u || request->write_quantity > 121u ||
            !address_range_is_valid(request->write_start_address,
                                    request->write_quantity) ||
            request->value != 0u) {
            return MBRTUM_ERROR_QUANTITY;
        }
        break;
    case MBRTUM_FC_READ_EXCEPTION_STATUS:
    case MBRTUM_FC_GET_COMM_EVENT_COUNTER:
    case MBRTUM_FC_GET_COMM_EVENT_LOG:
        if (request->start_address != 0u || request->quantity != 0u ||
            request->value != 0u) {
            return MBRTUM_ERROR_VALUE;
        }
        break;
    case MBRTUM_FC_READ_DEVICE_IDENTIFICATION:
        if (!device_id_code_is_valid(request->start_address) ||
            request->quantity > UINT8_MAX ||
            request->value != MB_DEVICE_ID_MEI_TYPE ||
            request->write_start_address != 0u ||
            request->write_quantity != 0u) {
            return MBRTUM_ERROR_VALUE;
        }
        break;
    case MBRTUM_FC_DIAGNOSTICS:
        if (!diagnostic_subfunction_is_supported(request->start_address) ||
            request->quantity > 250u ||
            (request->quantity % 2u) != 0u || request->value != 0u) {
            return MBRTUM_ERROR_VALUE;
        }
        if (request->start_address == MBRTU_DIAG_SUB_RETURN_QUERY_DATA) {
            break;
        }
        if (request->quantity != 2u) {
            return MBRTUM_ERROR_VALUE;
        }
        break;
    default:
        return MBRTUM_ERROR_FUNCTION;
    }

    return MBRTUM_OK;
}

static int original_request_adu_is_valid(
    const mbrtum_request_t *request,
    const uint8_t *request_adu,
    size_t request_adu_length)
{
    if (request_adu == NULL ||
        request_adu_length < MODBUS_RTU_ADU_MIN_SIZE ||
        request_adu_length > MODBUS_RTU_ADU_MAX_SIZE ||
        mb_crc16(request_adu, request_adu_length) != 0u ||
        request_adu[0] != request->slave_address ||
        request_adu[1] != request->function) {
        return 0;
    }

    if (request->function == MBRTUM_FC_DIAGNOSTICS) {
        uint16_t request_data = 0u;

        if (request_adu_length != 6u + (size_t)request->quantity ||
            read_be16(&request_adu[2]) != request->start_address) {
            return 0;
        }
        if (request->start_address == MBRTU_DIAG_SUB_RETURN_QUERY_DATA) {
            return 1;
        }
        request_data = read_be16(&request_adu[4]);
        if (request->start_address ==
            MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS) {
            return request_data == 0x0000u || request_data == 0xFF00u;
        }
        return request_data == 0u;
    }
    return 1;
}

int mbrtum_process_response_with_request_adu(
    const mbrtum_request_t *request,
    const uint8_t *request_adu,
    size_t request_adu_length,
    const uint8_t *response_adu,
    size_t response_adu_length,
    mbrtum_response_t *response)
{
    uint8_t response_function;
    uint8_t exception_function;
    int request_result;

    clear_response(response);
    if (request == NULL || response_adu == NULL || response == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }

    request_result = validate_request_descriptor(request);
    if (request_result != MBRTUM_OK) {
        return request_result;
    }
    if (request->function == MBRTUM_FC_DIAGNOSTICS &&
        !original_request_adu_is_valid(request,
                                       request_adu,
                                       request_adu_length)) {
        return MBRTUM_ERROR_REQUEST_DATA_REQUIRED;
    }
    if (response_adu_length < MBRTUM_EXCEPTION_ADU_SIZE ||
        response_adu_length > MODBUS_RTU_ADU_MAX_SIZE) {
        return MBRTUM_ERROR_RESPONSE_LENGTH;
    }
    if (mb_crc16(response_adu, response_adu_length) != 0u) {
        return MBRTUM_ERROR_CRC;
    }
    if (response_adu[0] != request->slave_address) {
        return MBRTUM_ERROR_ADDRESS_MISMATCH;
    }

    response_function = response_adu[1];
    exception_function = (uint8_t)(request->function | 0x80u);
    if (response_function == exception_function) {
        if (response_adu_length != MBRTUM_EXCEPTION_ADU_SIZE) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        if (response_adu[2] == 0u) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        response->function = request->function;
        response->exception_code = response_adu[2];
        return MBRTUM_EXCEPTION_RESPONSE;
    }
    if (response_function != request->function) {
        return MBRTUM_ERROR_FUNCTION_MISMATCH;
    }

    switch (request->function) {
    case MBRTUM_FC_READ_COILS:
    case MBRTUM_FC_READ_DISCRETE_INPUTS: {
        size_t expected_byte_count = ((size_t)request->quantity + 7u) / 8u;
        size_t expected_length = 5u + expected_byte_count;

        if (response_adu_length != expected_length) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        if ((size_t)response_adu[2] != expected_byte_count) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        if (!packed_bit_padding_is_zero(&response_adu[3],
                                        expected_byte_count,
                                        request->quantity)) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        response->function = request->function;
        response->data = &response_adu[3];
        response->data_length = expected_byte_count;
        return MBRTUM_OK;
    }

    case MBRTUM_FC_READ_HOLDING_REGISTERS:
    case MBRTUM_FC_READ_INPUT_REGISTERS:
    case MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS: {
        size_t expected_byte_count = (size_t)request->quantity * 2u;
        size_t expected_length = 5u + expected_byte_count;

        if (response_adu_length != expected_length) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        if ((size_t)response_adu[2] != expected_byte_count) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        response->function = request->function;
        response->data = &response_adu[3];
        response->data_length = expected_byte_count;
        return MBRTUM_OK;
    }

    case MBRTUM_FC_WRITE_SINGLE_COIL:
    case MBRTUM_FC_WRITE_SINGLE_REGISTER:
        if (response_adu_length != MBRTUM_WRITE_ACK_ADU_SIZE) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        if (read_be16(&response_adu[2]) != request->start_address ||
            read_be16(&response_adu[4]) != request->value) {
            return MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH;
        }
        response->function = request->function;
        return MBRTUM_OK;

    case MBRTUM_FC_WRITE_MULTIPLE_COILS:
    case MBRTUM_FC_WRITE_MULTIPLE_REGISTERS:
        if (response_adu_length != MBRTUM_WRITE_ACK_ADU_SIZE) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        if (read_be16(&response_adu[2]) != request->start_address ||
            read_be16(&response_adu[4]) != request->quantity) {
            return MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH;
        }
        response->function = request->function;
        return MBRTUM_OK;

    case MBRTUM_FC_READ_EXCEPTION_STATUS:
        if (response_adu_length != 5u) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        response->function = request->function;
        response->data = &response_adu[2];
        response->data_length = 1u;
        return MBRTUM_OK;

    case MBRTUM_FC_DIAGNOSTICS: {
        uint16_t response_subfunction;

        if (response_adu_length < 6u ||
            ((response_adu_length - 6u) % 2u) != 0u) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        response_subfunction = read_be16(&response_adu[2]);
        if (response_subfunction != request->start_address) {
            return MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH;
        }

        if (request->start_address ==
            MBRTU_DIAG_SUB_RETURN_QUERY_DATA) {
            if (response_adu_length != request_adu_length ||
                memcmp(response_adu,
                       request_adu,
                       request_adu_length) != 0) {
                return MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH;
            }
        } else if (response_adu_length != 8u) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        } else if (request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_DIAGNOSTIC_REGISTER &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_BUS_MESSAGE_COUNT &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_BUS_COMM_ERROR_COUNT &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_BUS_EXCEPTION_COUNT &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_SERVER_MESSAGE_COUNT &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_SERVER_NO_RESPONSE_COUNT &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_SERVER_NAK_COUNT &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_SERVER_BUSY_COUNT &&
                   request->start_address !=
                       MBRTU_DIAG_SUB_RETURN_BUS_OVERRUN_COUNT &&
                   memcmp(&response_adu[4],
                          &request_adu[4],
                          2u) != 0) {
            return MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH;
        }

        response->function = request->function;
        response->data = &response_adu[2];
        response->data_length = response_adu_length - 4u;
        return MBRTUM_OK;
    }

    case MBRTUM_FC_GET_COMM_EVENT_COUNTER:
        if (response_adu_length != 8u) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        response->function = request->function;
        response->data = &response_adu[2];
        response->data_length = 4u;
        return MBRTUM_OK;

    case MBRTUM_FC_GET_COMM_EVENT_LOG: {
        size_t byte_count;
        size_t expected_length;

        if (response_adu_length < 11u) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        byte_count = response_adu[2];
        if (byte_count < 6u || byte_count > 70u) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        expected_length = 5u + byte_count;
        if (response_adu_length != expected_length) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }

        response->function = request->function;
        response->data = &response_adu[3];
        response->data_length = byte_count;
        return MBRTUM_OK;
    }

    case MBRTUM_FC_READ_DEVICE_IDENTIFICATION: {
        uint8_t read_code;
        uint8_t conformity_level;
        uint8_t more_follows;
        uint8_t next_object_id;
        uint8_t object_count;
        uint8_t category;
        size_t offset;
        uint8_t previous_object_id = 0u;

        if (response_adu_length < 12u) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        if (response_adu[2] != MB_DEVICE_ID_MEI_TYPE) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        read_code = response_adu[3];
        conformity_level = response_adu[4];
        more_follows = response_adu[5];
        next_object_id = response_adu[6];
        object_count = response_adu[7];

        if (read_code != (uint8_t)request->start_address ||
            !device_id_conformity_is_valid(conformity_level) ||
            (more_follows != 0u && more_follows != 0xFFu) ||
            object_count == 0u) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        if ((more_follows == 0u && next_object_id != 0u) ||
            (more_follows == 0xFFu && next_object_id == 0u)) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }

        category = device_id_conformity_category(conformity_level);
        if (read_code != MB_DEVICE_ID_READ_SPECIFIC &&
            read_code < category) {
            category = read_code;
        }
        if (read_code == MB_DEVICE_ID_READ_SPECIFIC) {
            if ((conformity_level & 0x80u) == 0u || more_follows != 0u ||
                next_object_id != 0u || object_count != 1u) {
                return MBRTUM_ERROR_MALFORMED_RESPONSE;
            }
        }

        offset = 8u;
        for (uint8_t i = 0u; i < object_count; ++i) {
            uint8_t object_id;
            uint8_t object_length;

            if (offset + 2u > response_adu_length - 2u) {
                return MBRTUM_ERROR_RESPONSE_LENGTH;
            }
            object_id = response_adu[offset];
            object_length = response_adu[offset + 1u];
            offset += 2u;
            if (offset + object_length > response_adu_length - 2u) {
                return MBRTUM_ERROR_RESPONSE_LENGTH;
            }
            if (i > 0u && object_id <= previous_object_id) {
                return MBRTUM_ERROR_MALFORMED_RESPONSE;
            }
            if (read_code == MB_DEVICE_ID_READ_SPECIFIC) {
                if (!device_id_object_matches_category(object_id, category)) {
                    return MBRTUM_ERROR_MALFORMED_RESPONSE;
                }
                if (object_id != (uint8_t)request->quantity) {
                    return MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH;
                }
            } else {
                if (!device_id_object_matches_category(object_id, category)) {
                    return MBRTUM_ERROR_MALFORMED_RESPONSE;
                }
                if (i == 0u && object_id != 0u &&
                    object_id != (uint8_t)request->quantity) {
                    return MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH;
                }
            }
            previous_object_id = object_id;
            offset += object_length;
        }
        if (offset != response_adu_length - 2u) {
            return MBRTUM_ERROR_RESPONSE_LENGTH;
        }
        if (more_follows == 0xFFu &&
            (next_object_id <= previous_object_id ||
             !device_id_object_matches_category(next_object_id, category))) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }

        response->function = request->function;
        response->data = &response_adu[2];
        response->data_length = response_adu_length - 4u;
        return MBRTUM_OK;
    }

    default:
        return MBRTUM_ERROR_FUNCTION;
    }
}

int mbrtum_process_response(const mbrtum_request_t *request,
                            const uint8_t *response_adu,
                            size_t response_adu_length,
                            mbrtum_response_t *response)
{
    return mbrtum_process_response_with_request_adu(request,
                                                     NULL,
                                                     0u,
                                                     response_adu,
                                                     response_adu_length,
                                                     response);
}

int mbrtum_get_bit(const mbrtum_request_t *request,
                   const mbrtum_response_t *response,
                   uint16_t index,
                   uint8_t *value)
{
    size_t byte_index;

    if (request == NULL || response == NULL || value == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (request->function != MBRTUM_FC_READ_COILS &&
        request->function != MBRTUM_FC_READ_DISCRETE_INPUTS) {
        return MBRTUM_ERROR_FUNCTION;
    }
    if (response->exception_code != 0u ||
        response->function != request->function || response->data == NULL) {
        return MBRTUM_ERROR_MALFORMED_RESPONSE;
    }
    if (index >= request->quantity) {
        return MBRTUM_ERROR_INDEX;
    }

    byte_index = (size_t)index / 8u;
    if (byte_index >= response->data_length) {
        return MBRTUM_ERROR_MALFORMED_RESPONSE;
    }

    *value = (uint8_t)((response->data[byte_index] >> (index % 8u)) & 1u);
    return MBRTUM_OK;
}

int mbrtum_get_register(const mbrtum_request_t *request,
                        const mbrtum_response_t *response,
                        uint16_t index,
                        uint16_t *value)
{
    size_t byte_index;

    if (request == NULL || response == NULL || value == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (request->function != MBRTUM_FC_READ_HOLDING_REGISTERS &&
        request->function != MBRTUM_FC_READ_INPUT_REGISTERS &&
        request->function != MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS) {
        return MBRTUM_ERROR_FUNCTION;
    }
    if (response->exception_code != 0u ||
        response->function != request->function || response->data == NULL) {
        return MBRTUM_ERROR_MALFORMED_RESPONSE;
    }
    if (index >= request->quantity) {
        return MBRTUM_ERROR_INDEX;
    }

    byte_index = (size_t)index * 2u;
    if (byte_index + 1u >= response->data_length) {
        return MBRTUM_ERROR_MALFORMED_RESPONSE;
    }

    *value = read_be16(&response->data[byte_index]);
    return MBRTUM_OK;
}

int mbrtum_get_exception_status(const mbrtum_response_t *response,
                                uint8_t *status)
{
    if (response == NULL || status == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (response->function != MBRTUM_FC_READ_EXCEPTION_STATUS ||
        response->exception_code != 0u || response->data == NULL ||
        response->data_length != 1u) {
        return MBRTUM_ERROR_FUNCTION;
    }
    *status = response->data[0];
    return MBRTUM_OK;
}

int mbrtum_get_diagnostics_response(
    const mbrtum_response_t *response,
    mbrtum_diagnostics_response_t *diagnostics_response)
{
    if (response == NULL || diagnostics_response == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (response->function != MBRTUM_FC_DIAGNOSTICS ||
        response->exception_code != 0u || response->data == NULL ||
        response->data_length < 2u ||
        (response->data_length % 2u) != 0u) {
        return MBRTUM_ERROR_FUNCTION;
    }
    diagnostics_response->subfunction = read_be16(response->data);
    diagnostics_response->data = &response->data[2];
    diagnostics_response->data_length = response->data_length - 2u;
    return MBRTUM_OK;
}

int mbrtum_get_diagnostics_word(const mbrtum_response_t *response,
                                uint16_t *subfunction,
                                uint16_t *value)
{
    mbrtum_diagnostics_response_t diagnostics_response;
    int result;

    if (subfunction == NULL || value == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    result = mbrtum_get_diagnostics_response(response,
                                              &diagnostics_response);
    if (result != MBRTUM_OK) {
        return result;
    }
    if (diagnostics_response.data_length < 2u) {
        return MBRTUM_ERROR_MALFORMED_RESPONSE;
    }
    *subfunction = diagnostics_response.subfunction;
    *value = read_be16(diagnostics_response.data);
    return MBRTUM_OK;
}

int mbrtum_get_comm_event_counter(const mbrtum_response_t *response,
                                  uint16_t *communication_status,
                                  uint16_t *event_count)
{
    if (response == NULL || communication_status == NULL ||
        event_count == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (response->function != MBRTUM_FC_GET_COMM_EVENT_COUNTER ||
        response->exception_code != 0u || response->data == NULL ||
        response->data_length != 4u) {
        return MBRTUM_ERROR_FUNCTION;
    }
    *communication_status = read_be16(response->data);
    *event_count = read_be16(&response->data[2]);
    return MBRTUM_OK;
}

int mbrtum_get_comm_event_log(const mbrtum_response_t *response,
                              mbrtum_comm_event_log_t *event_log)
{
    if (response == NULL || event_log == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (response->function != MBRTUM_FC_GET_COMM_EVENT_LOG ||
        response->exception_code != 0u || response->data == NULL ||
        response->data_length < 6u ||
        response->data_length > 70u) {
        return MBRTUM_ERROR_FUNCTION;
    }
    event_log->communication_status = read_be16(response->data);
    event_log->event_count = read_be16(&response->data[2]);
    event_log->message_count = read_be16(&response->data[4]);
    event_log->events = &response->data[6];
    event_log->events_length = response->data_length - 6u;
    return MBRTUM_OK;
}

int mbrtum_get_diagnostic_event(const mbrtum_comm_event_log_t *event_log,
                                size_t index,
                                uint8_t *event)
{
    if (event_log == NULL || event == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    if (event_log->events == NULL || index >= event_log->events_length) {
        return MBRTUM_ERROR_INDEX;
    }
    *event = event_log->events[index];
    return MBRTUM_OK;
}

int mbrtum_get_device_id_response(
    const mbrtum_response_t *response,
    mbrtum_device_id_response_t *device_id_response)
{
    if (response == NULL || device_id_response == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    memset(device_id_response, 0, sizeof(*device_id_response));
    if (response->exception_code != 0u ||
        response->function != MBRTUM_FC_READ_DEVICE_IDENTIFICATION ||
        response->data == NULL || response->data_length < 8u ||
        response->data[0] != MB_DEVICE_ID_MEI_TYPE) {
        return MBRTUM_ERROR_MALFORMED_RESPONSE;
    }

    device_id_response->read_device_id_code = response->data[1];
    device_id_response->conformity_level = response->data[2];
    device_id_response->more_follows = response->data[3];
    device_id_response->next_object_id = response->data[4];
    device_id_response->object_count = response->data[5];
    device_id_response->objects = &response->data[6];
    device_id_response->objects_length = response->data_length - 6u;
    return MBRTUM_OK;
}

int mbrtum_get_device_id_object(
    const mbrtum_device_id_response_t *device_id_response,
    size_t index,
    mbrtum_device_id_object_t *object)
{
    size_t offset = 0u;

    if (device_id_response == NULL || object == NULL) {
        return MBRTUM_ERROR_ARGUMENT;
    }
    memset(object, 0, sizeof(*object));
    if (device_id_response->objects == NULL ||
        index >= device_id_response->object_count) {
        return MBRTUM_ERROR_INDEX;
    }

    for (size_t i = 0u; i <= index; ++i) {
        uint8_t object_length;

        if (offset + 2u > device_id_response->objects_length) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        object_length = device_id_response->objects[offset + 1u];
        if (offset + 2u + object_length >
            device_id_response->objects_length) {
            return MBRTUM_ERROR_MALFORMED_RESPONSE;
        }
        if (i == index) {
            object->object_id = device_id_response->objects[offset];
            object->value = &device_id_response->objects[offset + 2u];
            object->value_length = object_length;
            return MBRTUM_OK;
        }
        offset += 2u + object_length;
    }

    return MBRTUM_ERROR_INDEX;
}
