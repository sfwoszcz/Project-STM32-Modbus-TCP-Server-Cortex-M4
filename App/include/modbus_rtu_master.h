#ifndef MODBUS_RTU_MASTER_H
#define MODBUS_RTU_MASTER_H

#include "modbus_rtu.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Supported Modbus function codes. */
#define MBRTUM_FC_READ_COILS                       0x01u
#define MBRTUM_FC_READ_DISCRETE_INPUTS             0x02u
#define MBRTUM_FC_READ_HOLDING_REGISTERS           0x03u
#define MBRTUM_FC_READ_INPUT_REGISTERS             0x04u
#define MBRTUM_FC_WRITE_SINGLE_COIL                0x05u
#define MBRTUM_FC_WRITE_SINGLE_REGISTER            0x06u
#define MBRTUM_FC_READ_EXCEPTION_STATUS             MBRTU_FC_READ_EXCEPTION_STATUS
#define MBRTUM_FC_DIAGNOSTICS                       MBRTU_FC_DIAGNOSTICS
#define MBRTUM_FC_GET_COMM_EVENT_COUNTER            MBRTU_FC_GET_COMM_EVENT_COUNTER
#define MBRTUM_FC_GET_COMM_EVENT_LOG                MBRTU_FC_GET_COMM_EVENT_LOG
#define MBRTUM_FC_WRITE_MULTIPLE_COILS             0x0Fu
#define MBRTUM_FC_WRITE_MULTIPLE_REGISTERS         0x10u
#define MBRTUM_FC_READ_WRITE_MULTIPLE_REGISTERS    0x17u

/* Common Modbus exception codes returned by a remote slave. */
#define MBRTUM_EXCEPTION_ILLEGAL_FUNCTION          0x01u
#define MBRTUM_EXCEPTION_ILLEGAL_DATA_ADDRESS      0x02u
#define MBRTUM_EXCEPTION_ILLEGAL_DATA_VALUE        0x03u
#define MBRTUM_EXCEPTION_SERVER_FAILURE            0x04u
#define MBRTUM_EXCEPTION_ACKNOWLEDGE                0x05u
#define MBRTUM_EXCEPTION_SERVER_BUSY                0x06u
#define MBRTUM_EXCEPTION_NEGATIVE_ACKNOWLEDGE       0x07u
#define MBRTUM_EXCEPTION_MEMORY_PARITY_ERROR        0x08u
#define MBRTUM_EXCEPTION_GATEWAY_PATH_UNAVAILABLE  0x0Au
#define MBRTUM_EXCEPTION_GATEWAY_TARGET_FAILED     0x0Bu

/* Successful return values. */
#define MBRTUM_OK                                   0
#define MBRTUM_EXCEPTION_RESPONSE                   1

/* Error return values. */
#define MBRTUM_ERROR_ARGUMENT                      (-1)
#define MBRTUM_ERROR_SLAVE_ADDRESS                 (-2)
#define MBRTUM_ERROR_FUNCTION                      (-3)
#define MBRTUM_ERROR_QUANTITY                      (-4)
#define MBRTUM_ERROR_VALUE                         (-5)
#define MBRTUM_ERROR_CAPACITY                      (-6)
#define MBRTUM_ERROR_RESPONSE_LENGTH               (-7)
#define MBRTUM_ERROR_CRC                           (-8)
#define MBRTUM_ERROR_ADDRESS_MISMATCH              (-9)
#define MBRTUM_ERROR_FUNCTION_MISMATCH             (-10)
#define MBRTUM_ERROR_MALFORMED_RESPONSE            (-11)
#define MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH      (-12)
#define MBRTUM_ERROR_RESPONSE_NOT_EXPECTED         (-13)
#define MBRTUM_ERROR_INDEX                         (-14)
#define MBRTUM_ERROR_REQUEST_DATA_REQUIRED         (-15)

/**
 * Description of one generated Modbus RTU master request.
 *
 * The request descriptor is populated by a request-builder function and must
 * remain unchanged until the corresponding response has been validated.
 *
 * value contains the encoded FC05 coil value (0xFF00 or 0x0000) or the FC06
 * register value. For FC08, start_address stores the subfunction and quantity
 * stores the diagnostic data length in bytes. For FC23, start_address and
 * quantity describe the read range, while write_start_address and
 * write_quantity describe the write range. These fields remain zero for
 * FC07, FC0B, and FC0C.
 *
 * expects_response is zero for valid broadcast writes and for requests such
 * as FC08 Force Listen Only Mode that intentionally have no response.
 */
typedef struct {
    uint8_t slave_address;
    uint8_t function;
    uint16_t start_address;
    uint16_t quantity;
    uint16_t value;
    uint8_t expects_response;
    uint16_t write_start_address;
    uint16_t write_quantity;
} mbrtum_request_t;

/**
 * View of one validated Modbus RTU master response.
 *
 * data points into the caller-owned response ADU and remains valid only while
 * that ADU remains unchanged. For FC01-FC04 and FC23 normal responses, data
 * references the packed-bit or register data bytes after the byte-count field.
 *
 * For successful write acknowledgements and exception responses, data is NULL
 * and data_length is zero.
 *
 * exception_code is zero for a normal response and contains the remote Modbus
 * exception code when mbrtum_process_response() returns
 * MBRTUM_EXCEPTION_RESPONSE.
 */
typedef struct {
    uint8_t function;
    uint8_t exception_code;
    const uint8_t *data;
    size_t data_length;
} mbrtum_response_t;

/** Decoded zero-copy FC08 response view. */
typedef struct {
    uint16_t subfunction;
    const uint8_t *data;
    size_t data_length;
} mbrtum_diagnostics_response_t;

/** Decoded FC0C response view. */
typedef struct {
    uint16_t communication_status;
    uint16_t event_count;
    uint16_t message_count;
    const uint8_t *events;
    size_t events_length;
} mbrtum_comm_event_log_t;

/**
 * Storage and overlap requirements.
 *
 * The request descriptor, request ADU buffer, request ADU length object, and
 * any input value array supplied to a builder must refer to non-overlapping
 * storage. The request descriptor, response ADU, and response view supplied
 * to mbrtum_process_response() must also refer to non-overlapping storage.
 */

/**
 * Build an FC01 Read Coils or FC02 Read Discrete Inputs RTU request ADU.
 *
 * function must be MBRTUM_FC_READ_COILS or
 * MBRTUM_FC_READ_DISCRETE_INPUTS. slave_address must be 1-247.
 * quantity must be 1-2000.
 *
 * request_adu receives address, PDU, CRC low byte, and CRC high byte.
 * request_adu_length is set to zero before validation and receives the final
 * ADU length on success.
 */
int mbrtum_build_read_bits_request(uint8_t slave_address,
                                   uint8_t function,
                                   uint16_t start_address,
                                   uint16_t quantity,
                                   mbrtum_request_t *request,
                                   uint8_t *request_adu,
                                   size_t request_adu_capacity,
                                   size_t *request_adu_length);

/**
 * Build an FC03 Read Holding Registers or FC04 Read Input Registers RTU
 * request ADU.
 *
 * function must be MBRTUM_FC_READ_HOLDING_REGISTERS or
 * MBRTUM_FC_READ_INPUT_REGISTERS. slave_address must be 1-247.
 * quantity must be 1-125.
 */
int mbrtum_build_read_registers_request(uint8_t slave_address,
                                        uint8_t function,
                                        uint16_t start_address,
                                        uint16_t quantity,
                                        mbrtum_request_t *request,
                                        uint8_t *request_adu,
                                        size_t request_adu_capacity,
                                        size_t *request_adu_length);

/**
 * Build an FC05 Write Single Coil RTU request ADU.
 *
 * slave_address may be 0 for a broadcast or 1-247 for a unicast request.
 * value must be 0 or 1.
 */
int mbrtum_build_write_single_coil_request(uint8_t slave_address,
                                           uint16_t address,
                                           uint8_t value,
                                           mbrtum_request_t *request,
                                           uint8_t *request_adu,
                                           size_t request_adu_capacity,
                                           size_t *request_adu_length);

/**
 * Build an FC06 Write Single Holding Register RTU request ADU.
 *
 * slave_address may be 0 for a broadcast or 1-247 for a unicast request.
 */
int mbrtum_build_write_single_register_request(uint8_t slave_address,
                                               uint16_t address,
                                               uint16_t value,
                                               mbrtum_request_t *request,
                                               uint8_t *request_adu,
                                               size_t request_adu_capacity,
                                               size_t *request_adu_length);

/**
 * Build an FC0F Write Multiple Coils RTU request ADU.
 *
 * slave_address may be 0 for a broadcast or 1-247 for a unicast request.
 * quantity must be 1-1968.
 *
 * packed_values contains ceil(quantity / 8) bytes in Modbus least-significant
 * bit first order. The unused high bits in the final byte are cleared in the
 * generated request. packed_values must not overlap request_adu.
 */
int mbrtum_build_write_multiple_coils_request(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *packed_values,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);

/**
 * Build an FC10 Write Multiple Holding Registers RTU request ADU.
 *
 * slave_address may be 0 for a broadcast or 1-247 for a unicast request.
 * quantity must be 1-123. values contains quantity host-endian uint16_t
 * values, which are encoded in big-endian Modbus wire order.
 *
 * values must not overlap request_adu.
 */
int mbrtum_build_write_multiple_registers_request(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *values,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);

/**
 * Build an FC23 Read/Write Multiple Holding Registers RTU request ADU.
 *
 * slave_address must be 1-247. read_quantity must be 1-125 and
 * write_quantity must be 1-121. write_values contains write_quantity
 * host-endian uint16_t values encoded in big-endian Modbus wire order.
 * Both 16-bit address ranges are validated. Broadcast is not supported because
 * FC23 requires a read response. write_values must not overlap request_adu.
 */
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
    size_t *request_adu_length);

/**
 * Build an FC07 Read Exception Status request.
 */
int mbrtum_build_read_exception_status_request(
    uint8_t slave_address,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);

/**
 * Build an FC08 Diagnostics request.
 *
 * diagnostic_data_length must be even and no larger than 250 bytes. Read-only
 * subfunctions require a unicast address. Broadcast is accepted only for the
 * state-changing subfunctions 0001, 0004, 000A, and 0014.
 */
int mbrtum_build_diagnostics_request(
    uint8_t slave_address,
    uint16_t subfunction,
    const uint8_t *diagnostic_data,
    size_t diagnostic_data_length,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);

/** Build an FC0B Get Communication Event Counter request. */
int mbrtum_build_get_comm_event_counter_request(
    uint8_t slave_address,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);

/** Build an FC0C Get Communication Event Log request. */
int mbrtum_build_get_comm_event_log_request(
    uint8_t slave_address,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);

/**
 * Validate a response while also supplying the original request ADU.
 *
 * This form is required for FC08 so echoed diagnostic data can be compared
 * byte-for-byte. It also supports all ordinary requests, FC07/FC0B/FC0C, and
 * FC23.
 */
int mbrtum_process_response_with_request_adu(
    const mbrtum_request_t *request,
    const uint8_t *request_adu,
    size_t request_adu_length,
    const uint8_t *response_adu,
    size_t response_adu_length,
    mbrtum_response_t *response);

/**
 * Validate and decode exactly one complete Modbus RTU response ADU.
 *
 * The response is checked against the original request descriptor:
 *
 * - response length and CRC;
 * - slave address;
 * - normal or exception function code;
 * - exact read byte count and zero-valued unused packed-bit padding;
 * - exact write acknowledgement address, quantity, or value.
 *
 * For broadcast requests, MBRTUM_ERROR_RESPONSE_NOT_EXPECTED is returned.
 *
 * @return MBRTUM_OK for a normal validated response,
 *         MBRTUM_EXCEPTION_RESPONSE for a validated Modbus exception response,
 *         or one of the MBRTUM_ERROR_* values.
 */
int mbrtum_process_response(const mbrtum_request_t *request,
                            const uint8_t *response_adu,
                            size_t response_adu_length,
                            mbrtum_response_t *response);

/**
 * Read one decoded FC01 or FC02 bit from a validated normal response.
 *
 * index is zero-based within the quantity requested by request.
 */
int mbrtum_get_bit(const mbrtum_request_t *request,
                   const mbrtum_response_t *response,
                   uint16_t index,
                   uint8_t *value);

/**
 * Read one decoded FC03, FC04, or FC23 register from a validated normal
 * response.
 *
 * index is zero-based within the quantity requested by request.
 */
int mbrtum_get_register(const mbrtum_request_t *request,
                        const mbrtum_response_t *response,
                        uint16_t index,
                        uint16_t *value);

/** Decode a validated FC07 status byte. */
int mbrtum_get_exception_status(const mbrtum_response_t *response,
                                uint8_t *status);

/** Decode a validated FC08 reply into a zero-copy view. */
int mbrtum_get_diagnostics_response(
    const mbrtum_response_t *response,
    mbrtum_diagnostics_response_t *diagnostics_response);

/** Decode the subfunction and first response word of a validated FC08 reply. */
int mbrtum_get_diagnostics_word(const mbrtum_response_t *response,
                                uint16_t *subfunction,
                                uint16_t *value);

/** Decode a validated FC0B response. */
int mbrtum_get_comm_event_counter(const mbrtum_response_t *response,
                                  uint16_t *communication_status,
                                  uint16_t *event_count);

/** Decode a validated FC0C response into a zero-copy view. */
int mbrtum_get_comm_event_log(const mbrtum_response_t *response,
                              mbrtum_comm_event_log_t *event_log);

/** Read one newest-first event byte from a decoded FC0C view. */
int mbrtum_get_diagnostic_event(const mbrtum_comm_event_log_t *event_log,
                                size_t index,
                                uint8_t *event);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_MASTER_H */
