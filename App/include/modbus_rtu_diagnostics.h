#ifndef MODBUS_RTU_DIAGNOSTICS_H
#define MODBUS_RTU_DIAGNOSTICS_H

#include "modbus_pdu.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MBRTU_FC_READ_EXCEPTION_STATUS 0x07u
#define MBRTU_FC_DIAGNOSTICS 0x08u
#define MBRTU_FC_GET_COMM_EVENT_COUNTER 0x0Bu
#define MBRTU_FC_GET_COMM_EVENT_LOG 0x0Cu

#define MBRTU_DIAG_SUB_RETURN_QUERY_DATA 0x0000u
#define MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS 0x0001u
#define MBRTU_DIAG_SUB_RETURN_DIAGNOSTIC_REGISTER 0x0002u
#define MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY 0x0004u
#define MBRTU_DIAG_SUB_CLEAR_COUNTERS_AND_REGISTER 0x000Au
#define MBRTU_DIAG_SUB_RETURN_BUS_MESSAGE_COUNT 0x000Bu
#define MBRTU_DIAG_SUB_RETURN_BUS_COMM_ERROR_COUNT 0x000Cu
#define MBRTU_DIAG_SUB_RETURN_BUS_EXCEPTION_COUNT 0x000Du
#define MBRTU_DIAG_SUB_RETURN_SERVER_MESSAGE_COUNT 0x000Eu
#define MBRTU_DIAG_SUB_RETURN_SERVER_NO_RESPONSE_COUNT 0x000Fu
#define MBRTU_DIAG_SUB_RETURN_SERVER_NAK_COUNT 0x0010u
#define MBRTU_DIAG_SUB_RETURN_SERVER_BUSY_COUNT 0x0011u
#define MBRTU_DIAG_SUB_RETURN_BUS_OVERRUN_COUNT 0x0012u
#define MBRTU_DIAG_SUB_CLEAR_OVERRUN_COUNTER 0x0014u

#define MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY 64u

#define MBRTU_DIAGNOSTICS_PENDING_RESTART 0x01u
#define MBRTU_DIAGNOSTICS_PENDING_ENTER_LISTEN_ONLY 0x02u

#define MBRTU_DIAGNOSTICS_PDU_NOT_HANDLED 0
#define MBRTU_DIAGNOSTICS_PDU_RESPONSE 1
#define MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE 2
#define MBRTU_DIAGNOSTICS_PDU_ERROR (-1)

typedef enum {
    MBRTU_DIAGNOSTICS_ACTION_RESTART_COMMUNICATIONS = 1,
    MBRTU_DIAGNOSTICS_ACTION_FORCE_LISTEN_ONLY = 2,
    MBRTU_DIAGNOSTICS_ACTION_CLEAR_COUNTERS_AND_REGISTER = 3,
    MBRTU_DIAGNOSTICS_ACTION_CLEAR_OVERRUN_COUNTER = 4
} mbrtu_diagnostics_action_t;

/**
 * Authorize one state-changing diagnostics operation.
 *
 * Return nonzero to permit the operation or zero to reject it with Modbus
 * exception 03 (Illegal Data Value). A NULL callback denies every
 * state-changing operation. Read-only diagnostics never invoke the callback.
 */
typedef int (*mbrtu_diagnostics_policy_fn)(
    void *user_context,
    mbrtu_diagnostics_action_t action,
    uint16_t request_data);

/**
 * Fixed-capacity serial-line diagnostics state.
 *
 * All 16-bit counters saturate at 0xFFFF. The event log is a newest-first
 * logical ring: insertion at capacity deterministically discards the oldest
 * event. This object contains no pointers and requires no heap allocation.
 */
typedef struct {
    uint8_t exception_status;
    uint16_t diagnostic_register;
    uint16_t communication_status;

    uint16_t event_counter;
    uint16_t message_count;
    uint16_t bus_communication_error_count;
    uint16_t bus_exception_error_count;
    uint16_t server_message_count;
    uint16_t server_no_response_count;
    uint16_t server_nak_count;
    uint16_t server_busy_count;
    uint16_t bus_character_overrun_count;

    uint8_t event_log[MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY];
    uint8_t event_log_head;
    uint8_t event_log_count;
    uint8_t listen_only;
    uint8_t pending_actions;
} mbrtu_diagnostics_t;

void mbrtu_diagnostics_init(mbrtu_diagnostics_t *diagnostics);
void mbrtu_diagnostics_set_exception_status(mbrtu_diagnostics_t *diagnostics,
                                             uint8_t status);
void mbrtu_diagnostics_set_register(mbrtu_diagnostics_t *diagnostics,
                                    uint16_t value);
void mbrtu_diagnostics_set_communication_status(
    mbrtu_diagnostics_t *diagnostics,
    uint16_t status);
int mbrtu_diagnostics_is_listen_only(
    const mbrtu_diagnostics_t *diagnostics);
uint8_t mbrtu_diagnostics_take_pending_actions(
    mbrtu_diagnostics_t *diagnostics);
int mbrtu_diagnostics_get_event(const mbrtu_diagnostics_t *diagnostics,
                                size_t newest_first_index,
                                uint8_t *event);

/** Account a complete frame that failed CRC or RTU inter-character checks. */
void mbrtu_diagnostics_note_communication_error(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t broadcast,
    uint8_t character_overrun);

/**
 * Account a local receive-storage/character-overrun condition.
 *
 * The bus message and receive event are always recorded. The standard overrun
 * counter is incremented only when addressed_to_server is nonzero.
 */
void mbrtu_diagnostics_note_character_overrun(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t broadcast,
    uint8_t addressed_to_server);

/**
 * Process only FC07/FC08/FC0B/FC0C request PDUs.
 *
 * This entry point intentionally has no Modbus TCP integration. request_address
 * is the RTU address byte and may be zero for a broadcast. response_pdu and
 * request_pdu must not overlap.
 */
int mbrtu_diagnostics_process_pdu(
    mbrtu_diagnostics_t *diagnostics,
    mbrtu_diagnostics_policy_fn policy,
    void *policy_context,
    uint8_t request_address,
    const uint8_t *request_pdu,
    size_t request_pdu_len,
    uint8_t *response_pdu,
    size_t response_capacity,
    size_t *response_pdu_len);

/** Internal accounting hooks used by the RTU ADU wrapper. */
void mbrtu_diagnostics_note_bus_message(mbrtu_diagnostics_t *diagnostics,
                                        uint8_t broadcast);
void mbrtu_diagnostics_note_server_message(mbrtu_diagnostics_t *diagnostics);
void mbrtu_diagnostics_note_normal_completion(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t no_response,
    uint8_t exclude_from_event_counter);
void mbrtu_diagnostics_note_exception(mbrtu_diagnostics_t *diagnostics,
                                      uint8_t exception_code);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_DIAGNOSTICS_H */
