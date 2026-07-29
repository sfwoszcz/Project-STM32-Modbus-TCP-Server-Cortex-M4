#ifndef MODBUS_MEI_H
#define MODBUS_MEI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MB_MEI_FUNCTION_CODE 0x2Bu
#define MB_MEI_TYPE_CANOPEN_GENERAL_REFERENCE 0x0Du
#define MB_MEI_TYPE_READ_DEVICE_IDENTIFICATION 0x0Eu
#define MB_MEI_MAX_DATA_LENGTH 251u

#ifndef MB_MEI_MAX_HANDLERS
#define MB_MEI_MAX_HANDLERS 8u
#endif

#if MB_MEI_MAX_HANDLERS < 1
#error "MB_MEI_MAX_HANDLERS must be at least 1"
#endif

#define MB_MEI_OK 0
#define MB_MEI_ERROR_ARGUMENT (-1)
#define MB_MEI_ERROR_TYPE (-2)
#define MB_MEI_ERROR_CAPACITY (-3)
#define MB_MEI_ERROR_NOT_FOUND (-4)

#define MB_MEI_HANDLER_OK 0u
#define MB_MEI_EXCEPTION_ILLEGAL_FUNCTION 0x01u
#define MB_MEI_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02u
#define MB_MEI_EXCEPTION_ILLEGAL_DATA_VALUE 0x03u
#define MB_MEI_EXCEPTION_SERVER_FAILURE 0x04u
#define MB_MEI_EXCEPTION_ACKNOWLEDGE 0x05u
#define MB_MEI_EXCEPTION_SERVER_BUSY 0x06u
#define MB_MEI_EXCEPTION_NEGATIVE_ACKNOWLEDGE 0x07u
#define MB_MEI_EXCEPTION_MEMORY_PARITY_ERROR 0x08u
#define MB_MEI_EXCEPTION_GATEWAY_PATH_UNAVAILABLE 0x0Au
#define MB_MEI_EXCEPTION_GATEWAY_TARGET_FAILED 0x0Bu

/**
 * Application handler for one generic FC43 Encapsulated Interface type.
 *
 * request_data points to the bytes after the MEI type in the request PDU.
 * The handler writes only interface-specific response bytes; the library adds
 * function code 0x2B and echoes mei_type. response_data_length must be set to
 * the number of bytes written and must not exceed response_capacity.
 *
 * Return MB_MEI_HANDLER_OK for success or one standard Modbus exception code.
 * The callback is invoked without the library lock held.
 */
typedef uint8_t (*mb_mei_handler_fn)(void *context,
                                     uint8_t mei_type,
                                     const uint8_t *request_data,
                                     size_t request_data_length,
                                     uint8_t *response_data,
                                     size_t response_capacity,
                                     size_t *response_data_length);

/**
 * Register or replace an application handler for one MEI type.
 *
 * MEI 0x0D is intentionally unsupported by this project and MEI 0x0E is
 * reserved for the built-in Read Device Identification implementation.
 * Other MEI types are unregistered by default. Applications are responsible
 * for using only MEI type assignments appropriate for their deployment.
 */
int mb_mei_register_handler(uint8_t mei_type,
                            mb_mei_handler_fn handler,
                            void *context);

/** Remove one generic MEI handler. */
int mb_mei_unregister_handler(uint8_t mei_type);

/** Remove all generic MEI handlers. The built-in MEI 0x0E remains available. */
void mb_mei_clear_handlers(void);

/** Return nonzero when one generic MEI handler is registered. */
int mb_mei_is_registered(uint8_t mei_type);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_MEI_H */
