#ifndef MODBUS_DEVICE_ID_H
#define MODBUS_DEVICE_ID_H

#include "modbus_mei.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MB_DEVICE_ID_FUNCTION_CODE MB_MEI_FUNCTION_CODE
#define MB_DEVICE_ID_MEI_TYPE MB_MEI_TYPE_READ_DEVICE_IDENTIFICATION

#define MB_DEVICE_ID_READ_BASIC 0x01u
#define MB_DEVICE_ID_READ_REGULAR 0x02u
#define MB_DEVICE_ID_READ_EXTENDED 0x03u
#define MB_DEVICE_ID_READ_SPECIFIC 0x04u

#define MB_DEVICE_ID_CONFORMITY_BASIC_STREAM 0x01u
#define MB_DEVICE_ID_CONFORMITY_REGULAR_STREAM 0x02u
#define MB_DEVICE_ID_CONFORMITY_EXTENDED_STREAM 0x03u
#define MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL 0x81u
#define MB_DEVICE_ID_CONFORMITY_REGULAR_INDIVIDUAL 0x82u
#define MB_DEVICE_ID_CONFORMITY_EXTENDED_INDIVIDUAL 0x83u

#define MB_DEVICE_ID_OBJECT_VENDOR_NAME 0x00u
#define MB_DEVICE_ID_OBJECT_PRODUCT_CODE 0x01u
#define MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION 0x02u
#define MB_DEVICE_ID_OBJECT_VENDOR_URL 0x03u
#define MB_DEVICE_ID_OBJECT_PRODUCT_NAME 0x04u
#define MB_DEVICE_ID_OBJECT_MODEL_NAME 0x05u
#define MB_DEVICE_ID_OBJECT_USER_APPLICATION_NAME 0x06u
#define MB_DEVICE_ID_PRIVATE_OBJECT_MIN 0x80u

#ifndef MB_DEVICE_ID_MAX_OBJECTS
#define MB_DEVICE_ID_MAX_OBJECTS 32u
#endif

#ifndef MB_DEVICE_ID_VALUE_STORAGE_SIZE
#define MB_DEVICE_ID_VALUE_STORAGE_SIZE 512u
#endif

#define MB_DEVICE_ID_MAX_VALUE_LENGTH 244u

#define MB_DEVICE_ID_OK 0
#define MB_DEVICE_ID_ERROR_ARGUMENT (-1)
#define MB_DEVICE_ID_ERROR_CONFORMITY (-2)
#define MB_DEVICE_ID_ERROR_OBJECTS (-3)
#define MB_DEVICE_ID_ERROR_CAPACITY (-4)

/**
 * Application-supplied Device Identification object.
 *
 * Values are copied into fixed-capacity library storage by
 * mb_device_id_configure(); the input buffers do not need to remain valid.
 */
typedef struct {
    uint8_t object_id;
    const uint8_t *value;
    size_t value_length;
} mb_device_id_object_t;

/**
 * Configure the FC43/14 Device Identification object model.
 *
 * objects must be strictly ordered by object_id and must contain mandatory
 * objects 0x00, 0x01, and 0x02. Standard object IDs 0x07-0x7F are reserved
 * and rejected. Extended/private objects use IDs 0x80-0xFF. The selected
 * conformity level limits which object categories may be configured.
 *
 * The entire configuration is validated before replacing the active state.
 * Object values are copied into deterministic fixed-capacity storage. Call
 * this function after mb_init(), because mb_init() clears the configuration.
 */
int mb_device_id_configure(uint8_t conformity_level,
                           const mb_device_id_object_t *objects,
                           size_t object_count);

/** Clear the active FC43/14 configuration. */
void mb_device_id_clear(void);

/** Return nonzero when an FC43/14 configuration is active. */
int mb_device_id_is_configured(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_DEVICE_ID_H */
