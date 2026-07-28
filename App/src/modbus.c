#include "modbus.h"

#include "modbus_device_id.h"
#include "modbus_device_id_internal.h"
#include "modbus_file_record.h"
#include "modbus_file_record_internal.h"
#include "modbus_pdu.h"

#include <string.h>

#define MB_EX_ILLEGAL_DATA_ADDRESS 0x02u
#define MB_EX_ILLEGAL_DATA_VALUE 0x03u
#define MB_EX_SERVER_FAILURE 0x04u

#define MB_DEVICE_ID_RESPONSE_HEADER_SIZE 7u
#define MB_DEVICE_ID_OBJECT_HEADER_SIZE 2u
#define MB_DEVICE_ID_MORE_FOLLOWS 0xFFu

static uint8_t coils[MB_MAX_COILS];
static uint8_t discrete_inputs[MB_MAX_DISCRETE_INPUTS];
static uint16_t holding_registers[MB_MAX_HREGS];
static uint16_t input_registers[MB_MAX_IREGS];

typedef struct {
    uint8_t object_id;
    uint8_t value_length;
    size_t value_offset;
} mb_device_id_entry_t;

typedef struct {
    uint8_t configured;
    uint8_t conformity_level;
    size_t object_count;
    mb_device_id_entry_t objects[MB_DEVICE_ID_MAX_OBJECTS];
    uint8_t values[MB_DEVICE_ID_VALUE_STORAGE_SIZE];
} mb_device_id_state_t;

static mb_device_id_state_t device_id_state;

typedef struct {
    size_t file_count;
    mb_file_record_file_t files[MB_FILE_RECORD_MAX_FILES];
} mb_file_record_state_t;

static mb_file_record_state_t file_record_state;

static int conformity_level_is_valid(uint8_t conformity_level)
{
    return conformity_level == MB_DEVICE_ID_CONFORMITY_BASIC_STREAM ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_REGULAR_STREAM ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_EXTENDED_STREAM ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_REGULAR_INDIVIDUAL ||
           conformity_level == MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL;
}

static uint8_t conformity_category(uint8_t conformity_level)
{
    return (uint8_t)(conformity_level & 0x7Fu);
}

static int object_id_is_defined(uint8_t object_id)
{
    return object_id <= MB_DEVICE_ID_OBJECT_USER_APPLICATION_NAME ||
           object_id >= MB_DEVICE_ID_PRIVATE_OBJECT_MIN;
}

static int object_id_matches_category(uint8_t object_id, uint8_t category)
{
    if (category == MB_DEVICE_ID_READ_BASIC) {
        return object_id <= MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION;
    }
    if (category == MB_DEVICE_ID_READ_REGULAR) {
        return object_id <= MB_DEVICE_ID_OBJECT_USER_APPLICATION_NAME;
    }
    return category == MB_DEVICE_ID_READ_EXTENDED &&
           object_id_is_defined(object_id);
}

static int configuration_object_is_allowed(uint8_t object_id,
                                           uint8_t category)
{
    if (!object_id_is_defined(object_id)) {
        return 0;
    }
    if (category == MB_DEVICE_ID_READ_BASIC) {
        return object_id <= MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION;
    }
    if (category == MB_DEVICE_ID_READ_REGULAR) {
        return object_id <= MB_DEVICE_ID_OBJECT_USER_APPLICATION_NAME;
    }
    return category == MB_DEVICE_ID_READ_EXTENDED;
}

static void clear_device_id_unlocked(void)
{
    memset(&device_id_state, 0, sizeof(device_id_state));
}

static void clear_file_records_unlocked(void)
{
    memset(&file_record_state, 0, sizeof(file_record_state));
}

void mb_init(void)
{
    mb_lock();
    memset(coils, 0, sizeof(coils));
    memset(discrete_inputs, 0, sizeof(discrete_inputs));
    memset(holding_registers, 0, sizeof(holding_registers));
    memset(input_registers, 0, sizeof(input_registers));
    clear_device_id_unlocked();
    clear_file_records_unlocked();
    mb_unlock();
}

int mb_device_id_configure(uint8_t conformity_level,
                           const mb_device_id_object_t *objects,
                           size_t object_count)
{
    size_t total_value_length = 0u;
    uint8_t category;

    if (objects == NULL) {
        return MB_DEVICE_ID_ERROR_ARGUMENT;
    }
    if (!conformity_level_is_valid(conformity_level)) {
        return MB_DEVICE_ID_ERROR_CONFORMITY;
    }
    if (object_count < 3u || object_count > MB_DEVICE_ID_MAX_OBJECTS ||
        object_count > UINT8_MAX) {
        return MB_DEVICE_ID_ERROR_CAPACITY;
    }

    category = conformity_category(conformity_level);
    for (size_t i = 0u; i < object_count; ++i) {
        if (objects[i].value_length > MB_DEVICE_ID_MAX_VALUE_LENGTH ||
            (objects[i].value_length > 0u && objects[i].value == NULL) ||
            !configuration_object_is_allowed(objects[i].object_id, category)) {
            return MB_DEVICE_ID_ERROR_OBJECTS;
        }
        if (i > 0u && objects[i - 1u].object_id >= objects[i].object_id) {
            return MB_DEVICE_ID_ERROR_OBJECTS;
        }
        if (objects[i].value_length >
            MB_DEVICE_ID_VALUE_STORAGE_SIZE - total_value_length) {
            return MB_DEVICE_ID_ERROR_CAPACITY;
        }
        total_value_length += objects[i].value_length;
    }

    if (objects[0].object_id != MB_DEVICE_ID_OBJECT_VENDOR_NAME ||
        objects[1].object_id != MB_DEVICE_ID_OBJECT_PRODUCT_CODE ||
        objects[2].object_id != MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION) {
        return MB_DEVICE_ID_ERROR_OBJECTS;
    }

    mb_lock();
    clear_device_id_unlocked();
    device_id_state.conformity_level = conformity_level;
    device_id_state.object_count = object_count;

    total_value_length = 0u;
    for (size_t i = 0u; i < object_count; ++i) {
        device_id_state.objects[i].object_id = objects[i].object_id;
        device_id_state.objects[i].value_length =
            (uint8_t)objects[i].value_length;
        device_id_state.objects[i].value_offset = total_value_length;
        if (objects[i].value_length > 0u) {
            memcpy(&device_id_state.values[total_value_length],
                   objects[i].value,
                   objects[i].value_length);
        }
        total_value_length += objects[i].value_length;
    }
    device_id_state.configured = 1u;
    mb_unlock();
    return MB_DEVICE_ID_OK;
}

void mb_device_id_clear(void)
{
    mb_lock();
    clear_device_id_unlocked();
    mb_unlock();
}

int mb_device_id_is_configured(void)
{
    int configured;

    mb_lock();
    configured = device_id_state.configured != 0u;
    mb_unlock();
    return configured;
}

static size_t find_object_index(uint8_t object_id,
                                uint8_t category,
                                int *found)
{
    for (size_t i = 0u; i < device_id_state.object_count; ++i) {
        if (!object_id_matches_category(device_id_state.objects[i].object_id,
                                        category)) {
            continue;
        }
        if (device_id_state.objects[i].object_id == object_id) {
            *found = 1;
            return i;
        }
    }

    *found = 0;
    return 0u;
}

static size_t first_eligible_index(uint8_t category)
{
    for (size_t i = 0u; i < device_id_state.object_count; ++i) {
        if (object_id_matches_category(device_id_state.objects[i].object_id,
                                       category)) {
            return i;
        }
    }
    return device_id_state.object_count;
}

static size_t next_eligible_index(size_t index, uint8_t category)
{
    for (size_t i = index + 1u; i < device_id_state.object_count; ++i) {
        if (object_id_matches_category(device_id_state.objects[i].object_id,
                                       category)) {
            return i;
        }
    }
    return device_id_state.object_count;
}

static size_t append_device_id_object(uint8_t *response_pdu,
                                      size_t offset,
                                      const mb_device_id_entry_t *entry)
{
    response_pdu[offset] = entry->object_id;
    response_pdu[offset + 1u] = entry->value_length;
    if (entry->value_length > 0u) {
        memcpy(&response_pdu[offset + MB_DEVICE_ID_OBJECT_HEADER_SIZE],
               &device_id_state.values[entry->value_offset],
               entry->value_length);
    }
    return offset + MB_DEVICE_ID_OBJECT_HEADER_SIZE + entry->value_length;
}

uint8_t mb_device_id_process_request(uint8_t read_device_id_code,
                                     uint8_t requested_object_id,
                                     uint8_t *response_pdu,
                                     size_t response_capacity,
                                     size_t *response_pdu_len)
{
    uint8_t actual_category;
    uint8_t requested_category;
    uint8_t response_category;
    size_t response_limit;
    size_t start_index;
    int found;

    if (response_pdu == NULL || response_pdu_len == NULL) {
        return MB_EX_SERVER_FAILURE;
    }
    *response_pdu_len = 0u;
    response_limit = response_capacity < MODBUS_PDU_MAX_SIZE
                         ? response_capacity
                         : MODBUS_PDU_MAX_SIZE;

    if (read_device_id_code < MB_DEVICE_ID_READ_BASIC ||
        read_device_id_code > MB_DEVICE_ID_READ_SPECIFIC) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    mb_lock();
    if (device_id_state.configured == 0u) {
        mb_unlock();
        return MB_EX_SERVER_FAILURE;
    }

    actual_category = conformity_category(device_id_state.conformity_level);
    if (read_device_id_code == MB_DEVICE_ID_READ_SPECIFIC) {
        const mb_device_id_entry_t *entry;
        size_t required_capacity;

        if ((device_id_state.conformity_level & 0x80u) == 0u) {
            mb_unlock();
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        start_index = find_object_index(requested_object_id,
                                        actual_category,
                                        &found);
        if (!found) {
            mb_unlock();
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }

        entry = &device_id_state.objects[start_index];
        required_capacity = MB_DEVICE_ID_RESPONSE_HEADER_SIZE +
                            MB_DEVICE_ID_OBJECT_HEADER_SIZE +
                            entry->value_length;
        if (response_limit < required_capacity) {
            mb_unlock();
            return MB_EX_SERVER_FAILURE;
        }

        response_pdu[0] = MB_DEVICE_ID_FUNCTION_CODE;
        response_pdu[1] = MB_DEVICE_ID_MEI_TYPE;
        response_pdu[2] = read_device_id_code;
        response_pdu[3] = device_id_state.conformity_level;
        response_pdu[4] = 0u;
        response_pdu[5] = 0u;
        response_pdu[6] = 1u;
        *response_pdu_len = append_device_id_object(
            response_pdu,
            MB_DEVICE_ID_RESPONSE_HEADER_SIZE,
            entry);
        mb_unlock();
        return 0u;
    }

    requested_category = read_device_id_code;
    response_category = requested_category < actual_category
                            ? requested_category
                            : actual_category;
    start_index = find_object_index(requested_object_id,
                                    response_category,
                                    &found);
    if (!found) {
        start_index = first_eligible_index(response_category);
    }
    if (start_index >= device_id_state.object_count ||
        response_limit < MB_DEVICE_ID_RESPONSE_HEADER_SIZE) {
        mb_unlock();
        return MB_EX_SERVER_FAILURE;
    }

    response_pdu[0] = MB_DEVICE_ID_FUNCTION_CODE;
    response_pdu[1] = MB_DEVICE_ID_MEI_TYPE;
    response_pdu[2] = read_device_id_code;
    response_pdu[3] = device_id_state.conformity_level;
    response_pdu[4] = 0u;
    response_pdu[5] = 0u;
    response_pdu[6] = 0u;

    {
        size_t offset = MB_DEVICE_ID_RESPONSE_HEADER_SIZE;
        size_t index = start_index;
        uint8_t object_count = 0u;

        while (index < device_id_state.object_count) {
            const mb_device_id_entry_t *entry =
                &device_id_state.objects[index];
            size_t object_size;
            size_t next_index;

            if (!object_id_matches_category(entry->object_id,
                                            response_category)) {
                index = next_eligible_index(index, response_category);
                continue;
            }

            object_size = MB_DEVICE_ID_OBJECT_HEADER_SIZE +
                          entry->value_length;
            if (object_size > response_limit - offset) {
                if (object_count == 0u) {
                    mb_unlock();
                    return MB_EX_SERVER_FAILURE;
                }
                response_pdu[4] = MB_DEVICE_ID_MORE_FOLLOWS;
                response_pdu[5] = entry->object_id;
                break;
            }

            offset = append_device_id_object(response_pdu, offset, entry);
            ++object_count;
            next_index = next_eligible_index(index, response_category);
            if (next_index >= device_id_state.object_count) {
                break;
            }
            index = next_index;
        }

        response_pdu[6] = object_count;
        *response_pdu_len = offset;
    }

    mb_unlock();
    return 0u;
}

int mb_file_record_configure(const mb_file_record_file_t *files,
                             size_t file_count)
{
    if (files == NULL) {
        return MB_FILE_RECORD_ERROR_ARGUMENT;
    }
    if (file_count == 0u || file_count > MB_FILE_RECORD_MAX_FILES) {
        return MB_FILE_RECORD_ERROR_CAPACITY;
    }

    for (size_t i = 0u; i < file_count; ++i) {
        if (files[i].file_number == 0u || files[i].records == NULL ||
            files[i].record_count == 0u ||
            files[i].record_count > MB_FILE_RECORD_MAX_RECORDS_PER_FILE) {
            return MB_FILE_RECORD_ERROR_FILES;
        }
        if (i > 0u &&
            files[i - 1u].file_number >= files[i].file_number) {
            return MB_FILE_RECORD_ERROR_FILES;
        }
    }

    mb_lock();
    clear_file_records_unlocked();
    memcpy(file_record_state.files,
           files,
           file_count * sizeof(file_record_state.files[0]));
    file_record_state.file_count = file_count;
    mb_unlock();
    return MB_FILE_RECORD_OK;
}

void mb_file_record_clear(void)
{
    mb_lock();
    clear_file_records_unlocked();
    mb_unlock();
}

int mb_file_record_is_configured(void)
{
    int configured;

    mb_lock();
    configured = file_record_state.file_count != 0u;
    mb_unlock();
    return configured;
}

static const mb_file_record_file_t *find_file_record_unlocked(
    uint16_t file_number)
{
    for (size_t i = 0u; i < file_record_state.file_count; ++i) {
        if (file_record_state.files[i].file_number == file_number) {
            return &file_record_state.files[i];
        }
        if (file_record_state.files[i].file_number > file_number) {
            break;
        }
    }
    return NULL;
}

uint8_t mb_file_record_process_read_request(const uint8_t *request_pdu,
                                            size_t request_pdu_len,
                                            uint8_t *response_pdu,
                                            size_t response_capacity,
                                            size_t *response_pdu_len)
{
    size_t request_data_length;
    size_t response_data_length = 0u;
    size_t response_limit;

    if (request_pdu == NULL || response_pdu == NULL ||
        response_pdu_len == NULL) {
        return MB_EX_SERVER_FAILURE;
    }
    *response_pdu_len = 0u;

    if (request_pdu_len < 2u) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    request_data_length = request_pdu[1];
    if (request_data_length < 7u ||
        request_data_length > MB_FILE_RECORD_REQUEST_DATA_MAX ||
        (request_data_length % 7u) != 0u ||
        request_pdu_len != 2u + request_data_length) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    response_limit = response_capacity < MODBUS_PDU_MAX_SIZE
                         ? response_capacity
                         : MODBUS_PDU_MAX_SIZE;

    mb_lock();
    for (size_t offset = 2u; offset < request_pdu_len; offset += 7u) {
        const mb_file_record_file_t *file;
        uint16_t file_number;
        uint16_t record_number;
        uint16_t record_length;
        size_t subresponse_length;

        if (request_pdu[offset] != MB_FILE_RECORD_REFERENCE_TYPE) {
            mb_unlock();
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        file_number = (uint16_t)(((uint16_t)request_pdu[offset + 1u] << 8u) |
                                 request_pdu[offset + 2u]);
        record_number =
            (uint16_t)(((uint16_t)request_pdu[offset + 3u] << 8u) |
                       request_pdu[offset + 4u]);
        record_length =
            (uint16_t)(((uint16_t)request_pdu[offset + 5u] << 8u) |
                       request_pdu[offset + 6u]);

        if (record_length == 0u) {
            mb_unlock();
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (record_number >= MB_FILE_RECORD_MAX_RECORDS_PER_FILE ||
            (uint32_t)record_number + (uint32_t)record_length >
                MB_FILE_RECORD_MAX_RECORDS_PER_FILE) {
            mb_unlock();
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }

        file = find_file_record_unlocked(file_number);
        if (file == NULL || (size_t)record_number > file->record_count ||
            (size_t)record_length >
                file->record_count - (size_t)record_number) {
            mb_unlock();
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }

        subresponse_length = 2u + ((size_t)record_length * 2u);
        if (subresponse_length >
                MB_FILE_RECORD_RESPONSE_DATA_MAX - response_data_length) {
            mb_unlock();
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        response_data_length += subresponse_length;
    }

    if (response_limit < 2u + response_data_length) {
        mb_unlock();
        return MB_EX_SERVER_FAILURE;
    }

    response_pdu[0] = 0x14u;
    response_pdu[1] = (uint8_t)response_data_length;
    {
        size_t response_offset = 2u;

        for (size_t request_offset = 2u;
             request_offset < request_pdu_len;
             request_offset += 7u) {
            const mb_file_record_file_t *file;
            uint16_t file_number;
            uint16_t record_number;
            uint16_t record_length;

            file_number =
                (uint16_t)(((uint16_t)request_pdu[request_offset + 1u] << 8u) |
                           request_pdu[request_offset + 2u]);
            record_number =
                (uint16_t)(((uint16_t)request_pdu[request_offset + 3u] << 8u) |
                           request_pdu[request_offset + 4u]);
            record_length =
                (uint16_t)(((uint16_t)request_pdu[request_offset + 5u] << 8u) |
                           request_pdu[request_offset + 6u]);
            file = find_file_record_unlocked(file_number);

            response_pdu[response_offset] =
                (uint8_t)(1u + ((size_t)record_length * 2u));
            response_pdu[response_offset + 1u] =
                MB_FILE_RECORD_REFERENCE_TYPE;
            response_offset += 2u;
            for (uint16_t i = 0u; i < record_length; ++i) {
                uint16_t value = file->records[(size_t)record_number + i];

                response_pdu[response_offset] = (uint8_t)(value >> 8u);
                response_pdu[response_offset + 1u] = (uint8_t)value;
                response_offset += 2u;
            }
        }
        *response_pdu_len = response_offset;
    }
    mb_unlock();
    return 0u;
}

uint8_t mb_get_coil(uint16_t address)
{
    return address < MB_MAX_COILS ? (uint8_t)(coils[address] & 1u) : 0u;
}

void mb_set_coil(uint16_t address, uint8_t value)
{
    uint8_t normalized;
    if (address >= MB_MAX_COILS) {
        return;
    }
    normalized = value != 0u ? 1u : 0u;
    mb_lock();
    coils[address] = normalized;
    mb_unlock();
    mb_on_write_coil(address, normalized);
}

uint8_t mb_get_dinput(uint16_t address)
{
    return address < MB_MAX_DISCRETE_INPUTS ? (uint8_t)(discrete_inputs[address] & 1u) : 0u;
}

void mb_set_dinput(uint16_t address, uint8_t value)
{
    if (address >= MB_MAX_DISCRETE_INPUTS) {
        return;
    }
    mb_lock();
    discrete_inputs[address] = value != 0u ? 1u : 0u;
    mb_unlock();
}

uint16_t mb_get_hreg(uint16_t address)
{
    return address < MB_MAX_HREGS ? holding_registers[address] : 0u;
}

void mb_set_hreg(uint16_t address, uint16_t value)
{
    if (address >= MB_MAX_HREGS) {
        return;
    }
    mb_lock();
    holding_registers[address] = value;
    mb_unlock();
    mb_on_write_hreg(address, value);
}

uint16_t mb_get_ireg(uint16_t address)
{
    return address < MB_MAX_IREGS ? input_registers[address] : 0u;
}

void mb_set_ireg(uint16_t address, uint16_t value)
{
    if (address >= MB_MAX_IREGS) {
        return;
    }
    mb_lock();
    input_registers[address] = value;
    mb_unlock();
}

void __attribute__((weak)) mb_on_write_hreg(uint16_t address, uint16_t value)
{
    (void)address;
    (void)value;
}

void __attribute__((weak)) mb_on_write_coil(uint16_t address, uint8_t value)
{
    (void)address;
    (void)value;
}
