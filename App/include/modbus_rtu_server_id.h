#ifndef MODBUS_RTU_SERVER_ID_H
#define MODBUS_RTU_SERVER_ID_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MBRTU_SERVER_ID_FUNCTION_CODE 0x11u
#define MBRTU_SERVER_ID_RUN_STATUS_OFF 0x00u
#define MBRTU_SERVER_ID_RUN_STATUS_ON 0xFFu
#define MBRTU_SERVER_ID_MAX_BYTE_COUNT 251u
#define MBRTU_SERVER_ID_MAX_SERVER_ID_LENGTH \
    (MBRTU_SERVER_ID_MAX_BYTE_COUNT - 1u)
#define MBRTU_SERVER_ID_MAX_ADDITIONAL_DATA \
    (MBRTU_SERVER_ID_MAX_BYTE_COUNT - 2u)

#define MBRTU_SERVER_ID_OK 0
#define MBRTU_SERVER_ID_ERROR_ARGUMENT (-1)
#define MBRTU_SERVER_ID_ERROR_STATUS (-2)
#define MBRTU_SERVER_ID_ERROR_CAPACITY (-3)
#define MBRTU_SERVER_ID_ERROR_LENGTH (-4)

/**
 * Configure the serial-line-only FC11 Report Server ID response.
 *
 * server_id_length is device-specific and must be from 1 through 250 bytes.
 * run_status must be MBRTU_SERVER_ID_RUN_STATUS_OFF or
 * MBRTU_SERVER_ID_RUN_STATUS_ON. additional_data may be NULL only when
 * additional_data_length is zero. The combined Server ID, Run Indicator
 * Status, and Additional Data must fit in the 251-byte FC11 byte-count field.
 *
 * Configuration data is copied into deterministic fixed-capacity storage, so
 * the caller's input buffers do not need to remain valid. Failed
 * configuration leaves the active response unchanged.
 */
int mbrtu_server_id_configure(const uint8_t *server_id,
                              size_t server_id_length,
                              uint8_t run_status,
                              const uint8_t *additional_data,
                              size_t additional_data_length);

/** Clear the configured FC11 response. */
void mbrtu_server_id_clear(void);

/** Return non-zero when an FC11 response is configured. */
int mbrtu_server_id_is_configured(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_SERVER_ID_H */
