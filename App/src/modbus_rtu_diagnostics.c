#include "modbus_rtu_diagnostics.h"

#if !defined(MBRTU_ENABLE_DIAGNOSTICS) || MBRTU_ENABLE_DIAGNOSTICS != 1
#error "Compile modbus_rtu_diagnostics.c with MBRTU_ENABLE_DIAGNOSTICS=1"
#endif

#include <limits.h>
#include <string.h>

#define MB_EX_ILLEGAL_FUNCTION 0x01u
#define MB_EX_ILLEGAL_DATA_VALUE 0x03u

#define MBRTU_DIAG_RECEIVE_EVENT 0x80u
#define MBRTU_DIAG_RECEIVE_COMM_ERROR 0x02u
#define MBRTU_DIAG_RECEIVE_CHARACTER_OVERRUN 0x10u
#define MBRTU_DIAG_RECEIVE_LISTEN_ONLY 0x20u
#define MBRTU_DIAG_RECEIVE_BROADCAST 0x40u

#define MBRTU_DIAG_SEND_EVENT 0x40u
#define MBRTU_DIAG_SEND_READ_EXCEPTION 0x01u
#define MBRTU_DIAG_SEND_SERVER_ABORT 0x02u
#define MBRTU_DIAG_SEND_SERVER_BUSY 0x04u
#define MBRTU_DIAG_SEND_SERVER_NAK 0x08u
#define MBRTU_DIAG_SEND_LISTEN_ONLY 0x20u

#define MBRTU_DIAG_ENTERED_LISTEN_ONLY_EVENT 0x04u
#define MBRTU_DIAG_RESTART_EVENT 0x00u

static uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8u) | (uint16_t)p[1]);
}

static void write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8u);
    p[1] = (uint8_t)value;
}

static void increment_saturating(uint16_t *value)
{
    if (*value != UINT16_MAX) {
        ++(*value);
    }
}

static void push_event(mbrtu_diagnostics_t *diagnostics, uint8_t event)
{
    if (diagnostics == NULL) {
        return;
    }

    if (diagnostics->event_log_count == 0u) {
        diagnostics->event_log_head = 0u;
    } else if (diagnostics->event_log_head == 0u) {
        diagnostics->event_log_head =
            (uint8_t)(MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY - 1u);
    } else {
        --diagnostics->event_log_head;
    }

    diagnostics->event_log[diagnostics->event_log_head] = event;
    if (diagnostics->event_log_count <
        MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY) {
        ++diagnostics->event_log_count;
    }
}

static uint8_t make_receive_event(const mbrtu_diagnostics_t *diagnostics,
                                  uint8_t broadcast,
                                  uint8_t communication_error,
                                  uint8_t character_overrun)
{
    uint8_t event = MBRTU_DIAG_RECEIVE_EVENT;

    if (broadcast != 0u) {
        event |= MBRTU_DIAG_RECEIVE_BROADCAST;
    }
    if (diagnostics != NULL && diagnostics->listen_only != 0u) {
        event |= MBRTU_DIAG_RECEIVE_LISTEN_ONLY;
    }
    if (communication_error != 0u) {
        event |= MBRTU_DIAG_RECEIVE_COMM_ERROR;
    }
    if (character_overrun != 0u) {
        event |= MBRTU_DIAG_RECEIVE_CHARACTER_OVERRUN;
    }
    return event;
}

static uint8_t make_send_event(const mbrtu_diagnostics_t *diagnostics,
                               uint8_t exception_code)
{
    uint8_t event = MBRTU_DIAG_SEND_EVENT;

    if (diagnostics != NULL && diagnostics->listen_only != 0u) {
        event |= MBRTU_DIAG_SEND_LISTEN_ONLY;
    }
    if (exception_code >= 1u && exception_code <= 3u) {
        event |= MBRTU_DIAG_SEND_READ_EXCEPTION;
    } else if (exception_code == 4u) {
        event |= MBRTU_DIAG_SEND_SERVER_ABORT;
    } else if (exception_code == 5u || exception_code == 6u) {
        event |= MBRTU_DIAG_SEND_SERVER_BUSY;
    } else if (exception_code == 7u) {
        event |= MBRTU_DIAG_SEND_SERVER_NAK;
    }
    return event;
}

static void clear_counters(mbrtu_diagnostics_t *diagnostics)
{
    diagnostics->event_counter = 0u;
    diagnostics->message_count = 0u;
    diagnostics->bus_communication_error_count = 0u;
    diagnostics->bus_exception_error_count = 0u;
    diagnostics->server_message_count = 0u;
    diagnostics->server_no_response_count = 0u;
    diagnostics->server_nak_count = 0u;
    diagnostics->server_busy_count = 0u;
    diagnostics->bus_character_overrun_count = 0u;
}

static int policy_allows(mbrtu_diagnostics_policy_fn policy,
                         void *policy_context,
                         mbrtu_diagnostics_action_t action,
                         uint16_t request_data)
{
    return policy != NULL &&
           policy(policy_context, action, request_data) != 0;
}

static int make_exception(uint8_t function,
                          uint8_t exception_code,
                          uint8_t *response_pdu,
                          size_t response_capacity,
                          size_t *response_pdu_len)
{
    if (response_capacity < 2u) {
        return MBRTU_DIAGNOSTICS_PDU_ERROR;
    }
    response_pdu[0] = (uint8_t)(function | 0x80u);
    response_pdu[1] = exception_code;
    *response_pdu_len = 2u;
    return MBRTU_DIAGNOSTICS_PDU_RESPONSE;
}

static int fixed_word_response(uint8_t function,
                               uint16_t subfunction,
                               uint16_t value,
                               uint8_t *response_pdu,
                               size_t response_capacity,
                               size_t *response_pdu_len)
{
    if (response_capacity < 5u) {
        return MBRTU_DIAGNOSTICS_PDU_ERROR;
    }
    response_pdu[0] = function;
    write_be16(&response_pdu[1], subfunction);
    write_be16(&response_pdu[3], value);
    *response_pdu_len = 5u;
    return MBRTU_DIAGNOSTICS_PDU_RESPONSE;
}

static int process_fc08(mbrtu_diagnostics_t *diagnostics,
                        mbrtu_diagnostics_policy_fn policy,
                        void *policy_context,
                        uint8_t request_address,
                        const uint8_t *request_pdu,
                        size_t request_pdu_len,
                        uint8_t *response_pdu,
                        size_t response_capacity,
                        size_t *response_pdu_len)
{
    uint16_t subfunction;
    uint16_t data = 0u;
    uint8_t broadcast = (uint8_t)(request_address == 0u ? 1u : 0u);

    if (request_pdu_len < 3u || ((request_pdu_len - 3u) % 2u) != 0u) {
        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        return make_exception(MBRTU_FC_DIAGNOSTICS,
                              MB_EX_ILLEGAL_DATA_VALUE,
                              response_pdu,
                              response_capacity,
                              response_pdu_len);
    }

    subfunction = read_be16(&request_pdu[1]);
    if (request_pdu_len >= 5u) {
        data = read_be16(&request_pdu[3]);
    }

    if (diagnostics->listen_only != 0u &&
        subfunction != MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS) {
        return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
    }

    switch (subfunction) {
    case MBRTU_DIAG_SUB_RETURN_QUERY_DATA:
        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        if (response_capacity < request_pdu_len) {
            return MBRTU_DIAGNOSTICS_PDU_ERROR;
        }
        memcpy(response_pdu, request_pdu, request_pdu_len);
        *response_pdu_len = request_pdu_len;
        return MBRTU_DIAGNOSTICS_PDU_RESPONSE;

    case MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS:
        if (request_pdu_len != 5u || (data != 0x0000u && data != 0xFF00u)) {
            if (broadcast != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (!policy_allows(policy,
                           policy_context,
                           MBRTU_DIAGNOSTICS_ACTION_RESTART_COMMUNICATIONS,
                           data)) {
            if (broadcast != 0u || diagnostics->listen_only != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics,
                    (uint8_t)(broadcast == 0u ? 1u : 0u),
                    1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (broadcast == 0u && diagnostics->listen_only == 0u &&
            response_capacity < 5u) {
            return MBRTU_DIAGNOSTICS_PDU_ERROR;
        }
        {
            uint8_t was_listen_only = diagnostics->listen_only;

            clear_counters(diagnostics);
            diagnostics->listen_only = 0u;
            diagnostics->pending_actions |= MBRTU_DIAGNOSTICS_PENDING_RESTART;
            if (data == 0xFF00u) {
                diagnostics->event_log_count = 0u;
                diagnostics->event_log_head = 0u;
            }
            push_event(diagnostics, MBRTU_DIAG_RESTART_EVENT);
            if (broadcast != 0u || was_listen_only != 0u) {
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return fixed_word_response(MBRTU_FC_DIAGNOSTICS,
                                       subfunction,
                                       data,
                                       response_pdu,
                                       response_capacity,
                                       response_pdu_len);
        }

    case MBRTU_DIAG_SUB_RETURN_DIAGNOSTIC_REGISTER:
        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        if (request_pdu_len != 5u || data != 0u) {
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        return fixed_word_response(MBRTU_FC_DIAGNOSTICS,
                                   subfunction,
                                   diagnostics->diagnostic_register,
                                   response_pdu,
                                   response_capacity,
                                   response_pdu_len);

    case MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY:
        if (request_pdu_len != 5u || data != 0u) {
            if (broadcast != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (!policy_allows(policy,
                           policy_context,
                           MBRTU_DIAGNOSTICS_ACTION_FORCE_LISTEN_ONLY,
                           data)) {
            if (broadcast != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        diagnostics->listen_only = 1u;
        mbrtu_diagnostics_note_normal_completion(
            diagnostics,
            (uint8_t)(broadcast == 0u ? 1u : 0u),
            0u);
        diagnostics->pending_actions |=
            MBRTU_DIAGNOSTICS_PENDING_ENTER_LISTEN_ONLY;
        push_event(diagnostics, MBRTU_DIAG_ENTERED_LISTEN_ONLY_EVENT);
        return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;

    case MBRTU_DIAG_SUB_CLEAR_COUNTERS_AND_REGISTER:
        if (request_pdu_len != 5u || data != 0u) {
            if (broadcast != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (!policy_allows(
                policy,
                policy_context,
                MBRTU_DIAGNOSTICS_ACTION_CLEAR_COUNTERS_AND_REGISTER,
                data)) {
            if (broadcast != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (broadcast == 0u && response_capacity < 5u) {
            return MBRTU_DIAGNOSTICS_PDU_ERROR;
        }
        clear_counters(diagnostics);
        diagnostics->diagnostic_register = 0u;
        mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
        if (broadcast != 0u) {
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        return fixed_word_response(MBRTU_FC_DIAGNOSTICS,
                                   subfunction,
                                   data,
                                   response_pdu,
                                   response_capacity,
                                   response_pdu_len);

    case MBRTU_DIAG_SUB_RETURN_BUS_MESSAGE_COUNT:
    case MBRTU_DIAG_SUB_RETURN_BUS_COMM_ERROR_COUNT:
    case MBRTU_DIAG_SUB_RETURN_BUS_EXCEPTION_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_MESSAGE_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_NO_RESPONSE_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_NAK_COUNT:
    case MBRTU_DIAG_SUB_RETURN_SERVER_BUSY_COUNT:
    case MBRTU_DIAG_SUB_RETURN_BUS_OVERRUN_COUNT: {
        uint16_t value;

        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        if (request_pdu_len != 5u || data != 0u) {
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        switch (subfunction) {
        case MBRTU_DIAG_SUB_RETURN_BUS_MESSAGE_COUNT:
            value = diagnostics->message_count;
            break;
        case MBRTU_DIAG_SUB_RETURN_BUS_COMM_ERROR_COUNT:
            value = diagnostics->bus_communication_error_count;
            break;
        case MBRTU_DIAG_SUB_RETURN_BUS_EXCEPTION_COUNT:
            value = diagnostics->bus_exception_error_count;
            break;
        case MBRTU_DIAG_SUB_RETURN_SERVER_MESSAGE_COUNT:
            value = diagnostics->server_message_count;
            break;
        case MBRTU_DIAG_SUB_RETURN_SERVER_NO_RESPONSE_COUNT:
            value = diagnostics->server_no_response_count;
            break;
        case MBRTU_DIAG_SUB_RETURN_SERVER_NAK_COUNT:
            value = diagnostics->server_nak_count;
            break;
        case MBRTU_DIAG_SUB_RETURN_SERVER_BUSY_COUNT:
            value = diagnostics->server_busy_count;
            break;
        default:
            value = diagnostics->bus_character_overrun_count;
            break;
        }
        return fixed_word_response(MBRTU_FC_DIAGNOSTICS,
                                   subfunction,
                                   value,
                                   response_pdu,
                                   response_capacity,
                                   response_pdu_len);
    }

    case MBRTU_DIAG_SUB_CLEAR_OVERRUN_COUNTER:
        if (request_pdu_len != 5u || data != 0u) {
            if (broadcast != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (!policy_allows(policy,
                           policy_context,
                           MBRTU_DIAGNOSTICS_ACTION_CLEAR_OVERRUN_COUNTER,
                           data)) {
            if (broadcast != 0u) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 1u);
                return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
            }
            return make_exception(MBRTU_FC_DIAGNOSTICS,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (broadcast == 0u && response_capacity < 5u) {
            return MBRTU_DIAGNOSTICS_PDU_ERROR;
        }
        diagnostics->bus_character_overrun_count = 0u;
        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(
                diagnostics, 0u, 0u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        return fixed_word_response(MBRTU_FC_DIAGNOSTICS,
                                   subfunction,
                                   data,
                                   response_pdu,
                                   response_capacity,
                                   response_pdu_len);

    default:
        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        return make_exception(MBRTU_FC_DIAGNOSTICS,
                              MB_EX_ILLEGAL_FUNCTION,
                              response_pdu,
                              response_capacity,
                              response_pdu_len);
    }
}

void mbrtu_diagnostics_init(mbrtu_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) {
        memset(diagnostics, 0, sizeof(*diagnostics));
    }
}

void mbrtu_diagnostics_set_exception_status(mbrtu_diagnostics_t *diagnostics,
                                             uint8_t status)
{
    if (diagnostics != NULL) {
        diagnostics->exception_status = status;
    }
}

void mbrtu_diagnostics_set_register(mbrtu_diagnostics_t *diagnostics,
                                    uint16_t value)
{
    if (diagnostics != NULL) {
        diagnostics->diagnostic_register = value;
    }
}

void mbrtu_diagnostics_set_communication_status(
    mbrtu_diagnostics_t *diagnostics,
    uint16_t status)
{
    if (diagnostics != NULL) {
        diagnostics->communication_status = status;
    }
}

int mbrtu_diagnostics_is_listen_only(
    const mbrtu_diagnostics_t *diagnostics)
{
    return diagnostics != NULL && diagnostics->listen_only != 0u;
}

uint8_t mbrtu_diagnostics_take_pending_actions(
    mbrtu_diagnostics_t *diagnostics)
{
    uint8_t actions;

    if (diagnostics == NULL) {
        return 0u;
    }
    actions = diagnostics->pending_actions;
    diagnostics->pending_actions = 0u;
    return actions;
}

int mbrtu_diagnostics_get_event(const mbrtu_diagnostics_t *diagnostics,
                                size_t newest_first_index,
                                uint8_t *event)
{
    size_t physical_index;

    if (diagnostics == NULL || event == NULL ||
        newest_first_index >= diagnostics->event_log_count) {
        return -1;
    }
    physical_index = ((size_t)diagnostics->event_log_head +
                      newest_first_index) %
                     MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY;
    *event = diagnostics->event_log[physical_index];
    return 0;
}

void mbrtu_diagnostics_note_bus_message(mbrtu_diagnostics_t *diagnostics,
                                        uint8_t broadcast)
{
    if (diagnostics == NULL) {
        return;
    }
    increment_saturating(&diagnostics->message_count);
    push_event(diagnostics,
               make_receive_event(diagnostics, broadcast, 0u, 0u));
}

void mbrtu_diagnostics_note_server_message(mbrtu_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) {
        increment_saturating(&diagnostics->server_message_count);
    }
}

void mbrtu_diagnostics_note_normal_completion(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t no_response,
    uint8_t exclude_from_event_counter)
{
    if (diagnostics == NULL) {
        return;
    }
    if (exclude_from_event_counter == 0u) {
        increment_saturating(&diagnostics->event_counter);
    }
    if (no_response != 0u) {
        increment_saturating(&diagnostics->server_no_response_count);
    }
    push_event(diagnostics, make_send_event(diagnostics, 0u));
}

void mbrtu_diagnostics_note_exception(mbrtu_diagnostics_t *diagnostics,
                                      uint8_t exception_code)
{
    if (diagnostics == NULL) {
        return;
    }
    increment_saturating(&diagnostics->bus_exception_error_count);
    if (exception_code == 6u) {
        increment_saturating(&diagnostics->server_busy_count);
    } else if (exception_code == 7u) {
        increment_saturating(&diagnostics->server_nak_count);
    }
    push_event(diagnostics,
               make_send_event(diagnostics, exception_code));
}

void mbrtu_diagnostics_note_communication_error(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t broadcast,
    uint8_t character_overrun)
{
    if (diagnostics == NULL) {
        return;
    }
    increment_saturating(&diagnostics->message_count);
    increment_saturating(&diagnostics->bus_communication_error_count);
    if (character_overrun != 0u) {
        increment_saturating(&diagnostics->bus_character_overrun_count);
    }
    push_event(diagnostics,
               make_receive_event(diagnostics,
                                  broadcast,
                                  1u,
                                  character_overrun));
}

void mbrtu_diagnostics_note_character_overrun(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t broadcast,
    uint8_t addressed_to_server)
{
    if (diagnostics == NULL) {
        return;
    }
    increment_saturating(&diagnostics->message_count);
    if (addressed_to_server != 0u) {
        increment_saturating(&diagnostics->bus_character_overrun_count);
    }
    push_event(diagnostics,
               make_receive_event(diagnostics,
                                  broadcast,
                                  0u,
                                  1u));
}

int mbrtu_diagnostics_process_pdu(
    mbrtu_diagnostics_t *diagnostics,
    mbrtu_diagnostics_policy_fn policy,
    void *policy_context,
    uint8_t request_address,
    const uint8_t *request_pdu,
    size_t request_pdu_len,
    uint8_t *response_pdu,
    size_t response_capacity,
    size_t *response_pdu_len)
{
    uint8_t function;
    uint8_t broadcast;

    if (response_pdu_len == NULL) {
        return MBRTU_DIAGNOSTICS_PDU_ERROR;
    }
    *response_pdu_len = 0u;
    if (diagnostics == NULL || request_pdu == NULL || response_pdu == NULL ||
        request_pdu_len == 0u || request_pdu_len > MODBUS_PDU_MAX_SIZE) {
        return MBRTU_DIAGNOSTICS_PDU_ERROR;
    }

    function = request_pdu[0];
    if (function != MBRTU_FC_READ_EXCEPTION_STATUS &&
        function != MBRTU_FC_DIAGNOSTICS &&
        function != MBRTU_FC_GET_COMM_EVENT_COUNTER &&
        function != MBRTU_FC_GET_COMM_EVENT_LOG) {
        return MBRTU_DIAGNOSTICS_PDU_NOT_HANDLED;
    }

    broadcast = (uint8_t)(request_address == 0u ? 1u : 0u);
    if (diagnostics->listen_only != 0u &&
        !(function == MBRTU_FC_DIAGNOSTICS && request_pdu_len >= 3u &&
          read_be16(&request_pdu[1]) ==
              MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS)) {
        return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
    }

    switch (function) {
    case MBRTU_FC_READ_EXCEPTION_STATUS:
        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        if (request_pdu_len != 1u) {
            return make_exception(function,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (response_capacity < 2u) {
            return MBRTU_DIAGNOSTICS_PDU_ERROR;
        }
        response_pdu[0] = function;
        response_pdu[1] = diagnostics->exception_status;
        *response_pdu_len = 2u;
        return MBRTU_DIAGNOSTICS_PDU_RESPONSE;

    case MBRTU_FC_DIAGNOSTICS:
        return process_fc08(diagnostics,
                            policy,
                            policy_context,
                            request_address,
                            request_pdu,
                            request_pdu_len,
                            response_pdu,
                            response_capacity,
                            response_pdu_len);

    case MBRTU_FC_GET_COMM_EVENT_COUNTER:
        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        if (request_pdu_len != 1u) {
            return make_exception(function,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        if (response_capacity < 5u) {
            return MBRTU_DIAGNOSTICS_PDU_ERROR;
        }
        response_pdu[0] = function;
        write_be16(&response_pdu[1], diagnostics->communication_status);
        write_be16(&response_pdu[3], diagnostics->event_counter);
        *response_pdu_len = 5u;
        return MBRTU_DIAGNOSTICS_PDU_RESPONSE;

    case MBRTU_FC_GET_COMM_EVENT_LOG: {
        size_t response_length;

        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE;
        }
        if (request_pdu_len != 1u) {
            return make_exception(function,
                                  MB_EX_ILLEGAL_DATA_VALUE,
                                  response_pdu,
                                  response_capacity,
                                  response_pdu_len);
        }
        response_length = 8u + diagnostics->event_log_count;
        if (response_capacity < response_length) {
            return MBRTU_DIAGNOSTICS_PDU_ERROR;
        }
        response_pdu[0] = function;
        response_pdu[1] = (uint8_t)(6u + diagnostics->event_log_count);
        write_be16(&response_pdu[2], diagnostics->communication_status);
        write_be16(&response_pdu[4], diagnostics->event_counter);
        write_be16(&response_pdu[6], diagnostics->message_count);
        for (size_t i = 0u; i < diagnostics->event_log_count; ++i) {
            uint8_t event = 0u;

            (void)mbrtu_diagnostics_get_event(diagnostics, i, &event);
            response_pdu[8u + i] = event;
        }
        *response_pdu_len = response_length;
        return MBRTU_DIAGNOSTICS_PDU_RESPONSE;
    }

    default:
        return MBRTU_DIAGNOSTICS_PDU_NOT_HANDLED;
    }
}
