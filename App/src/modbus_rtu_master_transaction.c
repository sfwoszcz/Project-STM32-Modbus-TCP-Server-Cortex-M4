#include "modbus_rtu_master_transaction.h"

#include "modbus_crc16.h"

#include <limits.h>
#include <string.h>

#define MBRTUM_TXN_INITIALIZATION_COOKIE UINT32_C(0x4D425458)
#define MBRTUM_TXN_MAX_SLAVE_ADDRESS 247u
#define MBRTUM_TXN_MIN_ADU_SIZE 4u

static void increment_saturating(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++(*value);
    }
}

static int deadline_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static int duration_is_valid(uint32_t duration)
{
    return duration <= (uint32_t)INT32_MAX;
}

static int state_is_active(mbrtum_transaction_state_t state)
{
    return state == MBRTUM_TXN_STATE_TRANSMITTING ||
           state == MBRTUM_TXN_STATE_WAITING_RESPONSE ||
           state == MBRTUM_TXN_STATE_RETRY_DELAY;
}

static int transaction_is_initialized(const mbrtum_transaction_t *transaction)
{
    return transaction != NULL &&
           transaction->initialization_cookie ==
               MBRTUM_TXN_INITIALIZATION_COOKIE;
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

static int request_adu_matches_descriptor(const mbrtum_request_t *request,
                                          const uint8_t *adu,
                                          size_t adu_length)
{
    uint16_t calculated_crc;

    if (request->slave_address > MBRTUM_TXN_MAX_SLAVE_ADDRESS ||
        adu_length < MBRTUM_TXN_MIN_ADU_SIZE) {
        return 0;
    }
    if (adu[0] != request->slave_address || adu[1] != request->function) {
        return 0;
    }
    if (request->slave_address == 0u && request->expects_response != 0u) {
        return 0;
    }

    if (request->function == MBRTUM_FC_DIAGNOSTICS) {
        uint16_t request_data = 0u;
        uint8_t expected_response = (uint8_t)(
            request->slave_address != MODBUS_RTU_BROADCAST_ADDRESS &&
            request->start_address != MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY);

        if (!diagnostic_subfunction_is_supported(request->start_address) ||
            request->quantity > 250u ||
            (request->quantity % 2u) != 0u || request->value != 0u ||
            adu_length != 6u + (size_t)request->quantity ||
            adu[2] != (uint8_t)(request->start_address >> 8u) ||
            adu[3] != (uint8_t)request->start_address ||
            request->expects_response != expected_response) {
            return 0;
        }
        if (request->slave_address == MODBUS_RTU_BROADCAST_ADDRESS &&
            !diagnostic_subfunction_is_state_changing(
                request->start_address)) {
            return 0;
        }
        if (request->start_address == MBRTU_DIAG_SUB_RETURN_QUERY_DATA) {
            /* The query-data payload may contain any even number of bytes. */
        } else {
            if (request->quantity != 2u) {
                return 0;
            }
            request_data = (uint16_t)(((uint16_t)adu[4] << 8u) |
                                      (uint16_t)adu[5]);
            if (request->start_address ==
                MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS) {
                if (request_data != 0x0000u && request_data != 0xFF00u) {
                    return 0;
                }
            } else if (request_data != 0u) {
                return 0;
            }
        }
    } else if (request->function ==
                   MBRTUM_FC_READ_EXCEPTION_STATUS ||
               request->function ==
                   MBRTUM_FC_GET_COMM_EVENT_COUNTER ||
               request->function ==
                   MBRTUM_FC_GET_COMM_EVENT_LOG) {
        if (request->slave_address == MODBUS_RTU_BROADCAST_ADDRESS ||
            request->expects_response != 1u ||
            request->start_address != 0u || request->quantity != 0u ||
            request->value != 0u ||
            adu_length != MBRTUM_TXN_MIN_ADU_SIZE) {
            return 0;
        }
    }

    calculated_crc = mb_crc16(adu, adu_length - 2u);
    return adu[adu_length - 2u] == (uint8_t)(calculated_crc & 0xFFu) &&
           adu[adu_length - 1u] == (uint8_t)(calculated_crc >> 8u);
}

static void clear_response(mbrtum_transaction_t *transaction)
{
    memset(transaction->response_adu, 0, sizeof(transaction->response_adu));
    transaction->response_adu_length = 0u;
    memset(&transaction->response, 0, sizeof(transaction->response));
    transaction->protocol_result = MBRTUM_TXN_PROTOCOL_NONE;
    transaction->exception_code = 0u;
}

static void finish(mbrtum_transaction_t *transaction,
                   mbrtum_transaction_state_t state,
                   mbrtum_transaction_result_t result)
{
    transaction->state = state;
    transaction->result = result;
}

static int begin_transmit(mbrtum_transaction_t *transaction, uint32_t now);

static void schedule_retry_or_finish(mbrtum_transaction_t *transaction,
                                     uint32_t now,
                                     mbrtum_transaction_result_t final_result)
{
    if (transaction->retries_used < transaction->config.max_retries) {
        ++transaction->retries_used;
        increment_saturating(&transaction->diagnostics.retries_performed);
        transaction->deadline = now + transaction->config.retry_delay_ticks;
        transaction->state = MBRTUM_TXN_STATE_RETRY_DELAY;
    } else {
        finish(transaction, MBRTUM_TXN_STATE_FAILED, final_result);
    }
}

static void handle_response_timeout(mbrtum_transaction_t *transaction,
                                    uint32_t now)
{
    mbrtum_transaction_result_t final_result;

    increment_saturating(&transaction->diagnostics.timeouts);
    final_result = transaction->saw_response_error != 0u
                       ? MBRTUM_TXN_RESULT_RESPONSE_ERROR
                       : MBRTUM_TXN_RESULT_TIMEOUT;
    schedule_retry_or_finish(transaction, now, final_result);
}

static int begin_transmit(mbrtum_transaction_t *transaction, uint32_t now)
{
    int transport_result;

    clear_response(transaction);
    transaction->saw_response_error = 0u;
    transaction->state = MBRTUM_TXN_STATE_TRANSMITTING;
    increment_saturating(&transaction->diagnostics.transmission_attempts);

    transport_result = transaction->transmit(transaction->transport_context,
                                             transaction->request_adu,
                                             transaction->request_adu_length);
    if (transport_result == MBRTUM_TXN_TRANSMIT_ACCEPTED) {
        return MBRTUM_TXN_OK;
    }

    increment_saturating(&transaction->diagnostics.transport_errors);
    if (transaction->config.retry_transport_errors != 0u) {
        schedule_retry_or_finish(transaction,
                                 now,
                                 MBRTUM_TXN_RESULT_TRANSPORT_ERROR);
    } else {
        finish(transaction,
               MBRTUM_TXN_STATE_FAILED,
               MBRTUM_TXN_RESULT_TRANSPORT_ERROR);
    }

    return MBRTUM_TXN_OK;
}

int mbrtum_transaction_init(mbrtum_transaction_t *transaction,
                            const mbrtum_transaction_config_t *config,
                            mbrtum_transaction_transmit_fn transmit,
                            void *transport_context)
{
    if (transaction == NULL || config == NULL || transmit == NULL) {
        return MBRTUM_TXN_ERROR_ARGUMENT;
    }
    if (config->response_timeout_ticks == 0u ||
        !duration_is_valid(config->response_timeout_ticks) ||
        !duration_is_valid(config->retry_delay_ticks) ||
        config->retry_transport_errors > 1u) {
        return MBRTUM_TXN_ERROR_CONFIG;
    }

    memset(transaction, 0, sizeof(*transaction));
    transaction->config = *config;
    transaction->transmit = transmit;
    transaction->transport_context = transport_context;
    transaction->state = MBRTUM_TXN_STATE_IDLE;
    transaction->result = MBRTUM_TXN_RESULT_NONE;
    transaction->initialization_cookie = MBRTUM_TXN_INITIALIZATION_COOKIE;
    return MBRTUM_TXN_OK;
}

int mbrtum_transaction_start(mbrtum_transaction_t *transaction,
                             const mbrtum_request_t *request,
                             const uint8_t *request_adu,
                             size_t request_adu_length,
                             uint32_t now)
{
    if (transaction == NULL || request == NULL || request_adu == NULL) {
        return MBRTUM_TXN_ERROR_ARGUMENT;
    }
    if (!transaction_is_initialized(transaction)) {
        return MBRTUM_TXN_ERROR_NOT_INITIALIZED;
    }
    if (state_is_active(transaction->state)) {
        return MBRTUM_TXN_ERROR_BUSY;
    }
    if (request_adu_length == 0u ||
        request_adu_length > sizeof(transaction->request_adu)) {
        return MBRTUM_TXN_ERROR_CAPACITY;
    }
    if (!request_adu_matches_descriptor(request,
                                        request_adu,
                                        request_adu_length)) {
        return MBRTUM_TXN_ERROR_REQUEST;
    }

    transaction->request = *request;
    memcpy(transaction->request_adu, request_adu, request_adu_length);
    transaction->request_adu_length = request_adu_length;
    transaction->retries_used = 0u;
    transaction->result = MBRTUM_TXN_RESULT_NONE;
    increment_saturating(&transaction->diagnostics.transactions_started);

    return begin_transmit(transaction, now);
}

int mbrtum_transaction_poll(mbrtum_transaction_t *transaction,
                            uint32_t now)
{
    if (transaction == NULL) {
        return MBRTUM_TXN_ERROR_ARGUMENT;
    }
    if (!transaction_is_initialized(transaction)) {
        return MBRTUM_TXN_ERROR_NOT_INITIALIZED;
    }

    if (transaction->state == MBRTUM_TXN_STATE_RETRY_DELAY) {
        if (deadline_reached(now, transaction->deadline)) {
            return begin_transmit(transaction, now);
        }
        return MBRTUM_TXN_OK;
    }

    if (transaction->state != MBRTUM_TXN_STATE_WAITING_RESPONSE) {
        return MBRTUM_TXN_OK;
    }

    if (!deadline_reached(now, transaction->deadline)) {
        return MBRTUM_TXN_OK;
    }

    handle_response_timeout(transaction, now);
    return MBRTUM_TXN_OK;
}

int mbrtum_transaction_on_tx_complete(mbrtum_transaction_t *transaction,
                                      uint32_t now)
{
    if (transaction == NULL) {
        return MBRTUM_TXN_ERROR_ARGUMENT;
    }
    if (!transaction_is_initialized(transaction)) {
        return MBRTUM_TXN_ERROR_NOT_INITIALIZED;
    }
    if (transaction->state != MBRTUM_TXN_STATE_TRANSMITTING) {
        return MBRTUM_TXN_ERROR_STATE;
    }

    if (transaction->request.expects_response == 0u) {
        increment_saturating(&transaction->diagnostics.successful_transactions);
        finish(transaction,
               MBRTUM_TXN_STATE_COMPLETE,
               MBRTUM_TXN_RESULT_SUCCESS);
        return MBRTUM_TXN_OK;
    }

    transaction->deadline = now + transaction->config.response_timeout_ticks;
    transaction->state = MBRTUM_TXN_STATE_WAITING_RESPONSE;
    return MBRTUM_TXN_OK;
}

int mbrtum_transaction_on_tx_error(mbrtum_transaction_t *transaction,
                                   uint32_t now)
{
    if (transaction == NULL) {
        return MBRTUM_TXN_ERROR_ARGUMENT;
    }
    if (!transaction_is_initialized(transaction)) {
        return MBRTUM_TXN_ERROR_NOT_INITIALIZED;
    }
    if (transaction->state != MBRTUM_TXN_STATE_TRANSMITTING) {
        return MBRTUM_TXN_ERROR_STATE;
    }

    increment_saturating(&transaction->diagnostics.transport_errors);
    if (transaction->config.retry_transport_errors != 0u) {
        schedule_retry_or_finish(transaction,
                                 now,
                                 MBRTUM_TXN_RESULT_TRANSPORT_ERROR);
    } else {
        finish(transaction,
               MBRTUM_TXN_STATE_FAILED,
               MBRTUM_TXN_RESULT_TRANSPORT_ERROR);
    }
    return MBRTUM_TXN_OK;
}

int mbrtum_transaction_on_response(mbrtum_transaction_t *transaction,
                                   const uint8_t *response_adu,
                                   size_t response_adu_length,
                                   uint32_t now)
{
    int result;

    if (transaction == NULL || response_adu == NULL) {
        return MBRTUM_TXN_ERROR_ARGUMENT;
    }
    if (!transaction_is_initialized(transaction)) {
        return MBRTUM_TXN_ERROR_NOT_INITIALIZED;
    }
    if (transaction->state != MBRTUM_TXN_STATE_WAITING_RESPONSE) {
        return MBRTUM_TXN_ERROR_STATE;
    }
    if (deadline_reached(now, transaction->deadline)) {
        increment_saturating(&transaction->diagnostics.late_responses);
        handle_response_timeout(transaction, now);
        return MBRTUM_TXN_OK;
    }
    if (response_adu_length == 0u ||
        response_adu_length > sizeof(transaction->response_adu)) {
        increment_saturating(&transaction->diagnostics.malformed_responses);
        transaction->saw_response_error = 1u;
        transaction->protocol_result = MBRTUM_ERROR_RESPONSE_LENGTH;
        return MBRTUM_TXN_OK;
    }

    memcpy(transaction->response_adu, response_adu, response_adu_length);
    transaction->response_adu_length = response_adu_length;
    memset(&transaction->response, 0, sizeof(transaction->response));

    result = mbrtum_process_response_with_request_adu(
        &transaction->request,
        transaction->request_adu,
        transaction->request_adu_length,
        transaction->response_adu,
        transaction->response_adu_length,
        &transaction->response);
    transaction->protocol_result = result;

    if (result == MBRTUM_OK) {
        increment_saturating(&transaction->diagnostics.successful_transactions);
        finish(transaction,
               MBRTUM_TXN_STATE_COMPLETE,
               MBRTUM_TXN_RESULT_SUCCESS);
        return MBRTUM_TXN_OK;
    }

    if (result == MBRTUM_EXCEPTION_RESPONSE) {
        transaction->exception_code = transaction->response.exception_code;
        increment_saturating(&transaction->diagnostics.exception_responses);
        finish(transaction,
               MBRTUM_TXN_STATE_COMPLETE,
               MBRTUM_TXN_RESULT_MODBUS_EXCEPTION);
        return MBRTUM_TXN_OK;
    }

    if (result == MBRTUM_ERROR_ADDRESS_MISMATCH ||
        result == MBRTUM_ERROR_FUNCTION_MISMATCH) {
        increment_saturating(&transaction->diagnostics.unrelated_responses);
        return MBRTUM_TXN_OK;
    }

    increment_saturating(&transaction->diagnostics.malformed_responses);
    transaction->saw_response_error = 1u;
    return MBRTUM_TXN_OK;
}

int mbrtum_transaction_cancel(mbrtum_transaction_t *transaction)
{
    if (transaction == NULL) {
        return MBRTUM_TXN_ERROR_ARGUMENT;
    }
    if (!transaction_is_initialized(transaction)) {
        return MBRTUM_TXN_ERROR_NOT_INITIALIZED;
    }
    if (!state_is_active(transaction->state)) {
        return MBRTUM_TXN_ERROR_STATE;
    }

    increment_saturating(&transaction->diagnostics.cancelled_transactions);
    finish(transaction,
           MBRTUM_TXN_STATE_CANCELLED,
           MBRTUM_TXN_RESULT_CANCELLED);
    return MBRTUM_TXN_OK;
}

int mbrtum_transaction_is_active(const mbrtum_transaction_t *transaction)
{
    if (!transaction_is_initialized(transaction)) {
        return 0;
    }
    return state_is_active(transaction->state);
}
