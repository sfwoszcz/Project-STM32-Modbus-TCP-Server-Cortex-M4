#include "modbus_rtu.h"

#include "modbus_crc16.h"
#include "modbus_file_record.h"
#include "modbus_pdu.h"
#include "modbus_rtu_diagnostics.h"
#include "modbus_rtu_server_id.h"
#include "modbus_rtu_server_id_internal.h"

#ifndef MBRTU_ENABLE_DIAGNOSTICS
#define MBRTU_ENABLE_DIAGNOSTICS 0
#endif

#ifndef MBRTU_ENABLE_SERVER_ID
#define MBRTU_ENABLE_SERVER_ID 0
#endif

#if MBRTU_ENABLE_DIAGNOSTICS == 0
/*
 * Keep the pre-diagnostics RTU source list link-compatible. Repository builds
 * define MBRTU_ENABLE_DIAGNOSTICS=1 and link modbus_rtu_diagnostics.c. Legacy
 * integrations that compile only modbus_rtu.c retain ordinary RTU behavior
 * without unresolved diagnostics symbols.
 */
static void diagnostics_disabled_note_communication_error(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t broadcast,
    uint8_t character_overrun)
{
    (void)diagnostics;
    (void)broadcast;
    (void)character_overrun;
}

static void diagnostics_disabled_note_character_overrun(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t broadcast,
    uint8_t addressed_to_server)
{
    (void)diagnostics;
    (void)broadcast;
    (void)addressed_to_server;
}

static void diagnostics_disabled_note_bus_message(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t broadcast)
{
    (void)diagnostics;
    (void)broadcast;
}

static void diagnostics_disabled_note_server_message(
    mbrtu_diagnostics_t *diagnostics)
{
    (void)diagnostics;
}

static void diagnostics_disabled_note_normal_completion(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t no_response,
    uint8_t exclude_from_event_counter)
{
    (void)diagnostics;
    (void)no_response;
    (void)exclude_from_event_counter;
}

static void diagnostics_disabled_note_exception(
    mbrtu_diagnostics_t *diagnostics,
    uint8_t exception_code)
{
    (void)diagnostics;
    (void)exception_code;
}

static int diagnostics_disabled_is_listen_only(
    const mbrtu_diagnostics_t *diagnostics)
{
    (void)diagnostics;
    return 0;
}

static int diagnostics_disabled_process_pdu(
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
    (void)diagnostics;
    (void)policy;
    (void)policy_context;
    (void)request_address;
    (void)request_pdu;
    (void)request_pdu_len;
    (void)response_pdu;
    (void)response_capacity;
    if (response_pdu_len != NULL) {
        *response_pdu_len = 0u;
    }
    return MBRTU_DIAGNOSTICS_PDU_NOT_HANDLED;
}

#define mbrtu_diagnostics_note_communication_error \
    diagnostics_disabled_note_communication_error
#define mbrtu_diagnostics_note_character_overrun \
    diagnostics_disabled_note_character_overrun
#define mbrtu_diagnostics_note_bus_message \
    diagnostics_disabled_note_bus_message
#define mbrtu_diagnostics_note_server_message \
    diagnostics_disabled_note_server_message
#define mbrtu_diagnostics_note_normal_completion \
    diagnostics_disabled_note_normal_completion
#define mbrtu_diagnostics_note_exception diagnostics_disabled_note_exception
#define mbrtu_diagnostics_is_listen_only diagnostics_disabled_is_listen_only
#define mbrtu_diagnostics_process_pdu diagnostics_disabled_process_pdu
#endif

#if MBRTU_ENABLE_SERVER_ID == 0
/* Preserve the pre-FC11 RTU source list when the optional source is absent. */
static int server_id_disabled_process_pdu(const uint8_t *request_pdu,
                                          size_t request_pdu_len,
                                          uint8_t *response_pdu,
                                          size_t response_capacity,
                                          size_t *response_pdu_len)
{
    (void)request_pdu;
    (void)request_pdu_len;
    (void)response_pdu;
    (void)response_capacity;
    if (response_pdu_len != NULL) {
        *response_pdu_len = 0u;
    }
    return MBRTU_SERVER_ID_PDU_NOT_HANDLED;
}
#define mbrtu_server_id_process_pdu server_id_disabled_process_pdu
#endif

static uint16_t read_crc_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8u));
}

static void write_crc_le(uint8_t *p, uint16_t crc)
{
    p[0] = (uint8_t)crc;
    p[1] = (uint8_t)(crc >> 8u);
}

static int is_supported_write_function(uint8_t function)
{
    return function == 0x05u || function == 0x06u ||
           function == 0x0Fu || function == 0x10u ||
           function == MB_FILE_RECORD_WRITE_FUNCTION_CODE;
}

static uint8_t response_exception_code(const uint8_t *response_pdu,
                                       size_t response_pdu_len)
{
    if (response_pdu_len == 2u && (response_pdu[0] & 0x80u) != 0u) {
        return response_pdu[1];
    }
    return 0u;
}

int mbrtu_process_adu_with_diagnostics(
    uint8_t slave_address,
    const uint8_t *request_adu,
    size_t request_adu_len,
    uint8_t *response_adu,
    size_t response_capacity,
    size_t *response_adu_len,
    mbrtu_diagnostics_t *diagnostics,
    mbrtu_diagnostics_policy_fn diagnostics_policy,
    void *diagnostics_policy_context)
{
    uint8_t request_address;
    uint8_t broadcast;
    size_t request_pdu_len;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (response_adu_len == NULL) {
        return MBRTU_ERROR_ARGUMENT;
    }
    *response_adu_len = 0u;

    if (request_adu == NULL || response_adu == NULL) {
        return MBRTU_ERROR_ARGUMENT;
    }

    if (slave_address < MODBUS_RTU_ADDRESS_MIN ||
        slave_address > MODBUS_RTU_ADDRESS_MAX) {
        return MBRTU_ERROR_SLAVE_ADDRESS;
    }

    if (request_adu_len < MODBUS_RTU_ADU_MIN_SIZE ||
        request_adu_len > MODBUS_RTU_ADU_MAX_SIZE) {
        if (diagnostics != NULL && request_adu_len > 0u) {
            mbrtu_diagnostics_note_communication_error(
                diagnostics,
                (uint8_t)(request_adu[0] == MODBUS_RTU_BROADCAST_ADDRESS),
                0u);
        }
        return MBRTU_NO_RESPONSE;
    }

    request_address = request_adu[0];
    broadcast = (uint8_t)(
        request_address == MODBUS_RTU_BROADCAST_ADDRESS ? 1u : 0u);

    received_crc = read_crc_le(&request_adu[request_adu_len - MODBUS_RTU_CRC_SIZE]);
    calculated_crc = mb_crc16(request_adu,
                              request_adu_len - MODBUS_RTU_CRC_SIZE);
    if (received_crc != calculated_crc) {
        mbrtu_diagnostics_note_communication_error(diagnostics,
                                                    broadcast,
                                                    0u);
        return MBRTU_NO_RESPONSE;
    }

    mbrtu_diagnostics_note_bus_message(diagnostics, broadcast);

    if (request_address != MODBUS_RTU_BROADCAST_ADDRESS &&
        request_address != slave_address) {
        return MBRTU_NO_RESPONSE;
    }

    request_pdu_len = request_adu_len - 1u - MODBUS_RTU_CRC_SIZE;
    mbrtu_diagnostics_note_server_message(diagnostics);

    if (diagnostics != NULL &&
        mbrtu_diagnostics_is_listen_only(diagnostics) &&
        !(request_pdu_len >= 5u &&
          request_adu[1] == MBRTU_FC_DIAGNOSTICS &&
          request_adu[2] == 0x00u &&
          request_adu[3] ==
              (uint8_t)MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS)) {
        mbrtu_diagnostics_note_normal_completion(
            diagnostics,
            (uint8_t)(broadcast == 0u ? 1u : 0u),
            1u);
        return MBRTU_NO_RESPONSE;
    }

    if (diagnostics != NULL) {
        uint8_t *diagnostic_response =
            response_capacity > 1u ? &response_adu[1] : response_adu;
        size_t diagnostic_response_capacity =
            response_capacity > 1u + MODBUS_RTU_CRC_SIZE
                ? response_capacity - 1u - MODBUS_RTU_CRC_SIZE
                : 0u;
        size_t diagnostic_response_len = 0u;
        int diagnostic_result;

        diagnostic_result = mbrtu_diagnostics_process_pdu(
            diagnostics,
            diagnostics_policy,
            diagnostics_policy_context,
            request_address,
            &request_adu[1],
            request_pdu_len,
            diagnostic_response,
            diagnostic_response_capacity,
            &diagnostic_response_len);

        if (diagnostic_result == MBRTU_DIAGNOSTICS_PDU_RESPONSE) {
            uint8_t exception_code;

            if (response_capacity <
                1u + diagnostic_response_len + MODBUS_RTU_CRC_SIZE) {
                return MBRTU_ERROR_RESPONSE_CAPACITY;
            }
            response_adu[0] = request_address;
            calculated_crc = mb_crc16(response_adu,
                                      1u + diagnostic_response_len);
            write_crc_le(&response_adu[1u + diagnostic_response_len],
                         calculated_crc);
            *response_adu_len =
                1u + diagnostic_response_len + MODBUS_RTU_CRC_SIZE;

            exception_code = response_exception_code(&response_adu[1],
                                                     diagnostic_response_len);
            if (exception_code != 0u) {
                mbrtu_diagnostics_note_exception(diagnostics,
                                                 exception_code);
            } else if (!(request_adu[1] == MBRTU_FC_DIAGNOSTICS &&
                         request_pdu_len >= 3u &&
                         request_adu[2] == 0x00u &&
                         (request_adu[3] ==
                              (uint8_t)MBRTU_DIAG_SUB_RESTART_COMMUNICATIONS ||
                          request_adu[3] ==
                              (uint8_t)MBRTU_DIAG_SUB_CLEAR_COUNTERS_AND_REGISTER))) {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics,
                    0u,
                    (uint8_t)(request_adu[1] ==
                                  MBRTU_FC_GET_COMM_EVENT_COUNTER ||
                              request_adu[1] ==
                                  MBRTU_FC_GET_COMM_EVENT_LOG));
            }
            return MBRTU_RESPONSE_READY;
        }

        if (diagnostic_result == MBRTU_DIAGNOSTICS_PDU_NO_RESPONSE) {
            return MBRTU_NO_RESPONSE;
        }

        if (diagnostic_result == MBRTU_DIAGNOSTICS_PDU_ERROR) {
            return MBRTU_ERROR_PDU;
        }
    }

    if (request_adu[1] == MBRTU_SERVER_ID_FUNCTION_CODE) {
        uint8_t *server_id_response =
            response_capacity > 1u ? &response_adu[1] : response_adu;
        size_t server_id_response_capacity =
            response_capacity > 1u + MODBUS_RTU_CRC_SIZE
                ? response_capacity - 1u - MODBUS_RTU_CRC_SIZE
                : 0u;
        size_t server_id_response_len = 0u;
        int server_id_result;

        if (broadcast != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_NO_RESPONSE;
        }

        server_id_result = mbrtu_server_id_process_pdu(
            &request_adu[1],
            request_pdu_len,
            server_id_response,
            server_id_response_capacity,
            &server_id_response_len);
        if (server_id_result == MBRTU_SERVER_ID_PDU_RESPONSE) {
            uint8_t exception_code;

            if (response_capacity <
                1u + server_id_response_len + MODBUS_RTU_CRC_SIZE) {
                return MBRTU_ERROR_RESPONSE_CAPACITY;
            }
            response_adu[0] = request_address;
            calculated_crc = mb_crc16(response_adu,
                                      1u + server_id_response_len);
            write_crc_le(&response_adu[1u + server_id_response_len],
                         calculated_crc);
            *response_adu_len =
                1u + server_id_response_len + MODBUS_RTU_CRC_SIZE;

            exception_code = response_exception_code(
                &response_adu[1], server_id_response_len);
            if (exception_code != 0u) {
                mbrtu_diagnostics_note_exception(diagnostics,
                                                 exception_code);
            } else {
                mbrtu_diagnostics_note_normal_completion(
                    diagnostics, 0u, 0u);
            }
            return MBRTU_RESPONSE_READY;
        }
        if (server_id_result == MBRTU_SERVER_ID_PDU_CAPACITY) {
            return MBRTU_ERROR_RESPONSE_CAPACITY;
        }
        if (server_id_result == MBRTU_SERVER_ID_PDU_ERROR) {
            return MBRTU_ERROR_PDU;
        }
    }

    if (request_address == MODBUS_RTU_BROADCAST_ADDRESS) {
        uint8_t discarded_response[MODBUS_PDU_MAX_SIZE];
        size_t discarded_response_len = 0u;
        int pdu_result;

        if (!is_supported_write_function(request_adu[1])) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
            return MBRTU_NO_RESPONSE;
        }

        pdu_result = mb_process_pdu(&request_adu[1],
                                    request_pdu_len,
                                    discarded_response,
                                    sizeof(discarded_response),
                                    &discarded_response_len);
        if (pdu_result < 0) {
            return MBRTU_ERROR_PDU;
        }
        if (response_exception_code(discarded_response,
                                    discarded_response_len) != 0u) {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 1u);
        } else {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 0u);
        }
        return MBRTU_NO_RESPONSE;
    }

    if (response_capacity < 1u + 2u + MODBUS_RTU_CRC_SIZE) {
        return MBRTU_ERROR_RESPONSE_CAPACITY;
    }

    {
        size_t response_pdu_len = 0u;
        size_t response_without_crc_len;
        uint8_t exception_code;
        int pdu_result;

        pdu_result = mb_process_pdu(&request_adu[1],
                                    request_pdu_len,
                                    &response_adu[1],
                                    response_capacity - 1u - MODBUS_RTU_CRC_SIZE,
                                    &response_pdu_len);
        if (pdu_result < 0) {
            return MBRTU_ERROR_PDU;
        }

        response_adu[0] = request_address;
        response_without_crc_len = 1u + response_pdu_len;
        calculated_crc = mb_crc16(response_adu, response_without_crc_len);
        write_crc_le(&response_adu[response_without_crc_len], calculated_crc);
        *response_adu_len = response_without_crc_len + MODBUS_RTU_CRC_SIZE;

        exception_code = response_exception_code(&response_adu[1],
                                                 response_pdu_len);
        if (exception_code != 0u) {
            mbrtu_diagnostics_note_exception(diagnostics, exception_code);
        } else {
            mbrtu_diagnostics_note_normal_completion(diagnostics, 0u, 0u);
        }
    }

    return MBRTU_RESPONSE_READY;
}

int mbrtu_process_adu(uint8_t slave_address,
                      const uint8_t *request_adu,
                      size_t request_adu_len,
                      uint8_t *response_adu,
                      size_t response_capacity,
                      size_t *response_adu_len)
{
    return mbrtu_process_adu_with_diagnostics(slave_address,
                                               request_adu,
                                               request_adu_len,
                                               response_adu,
                                               response_capacity,
                                               response_adu_len,
                                               NULL,
                                               NULL,
                                               NULL);
}

static uint32_t ceil_div_u64(uint64_t numerator, uint64_t denominator)
{
    return (uint32_t)((numerator + denominator - 1u) / denominator);
}

int mbrtu_calculate_timing(uint32_t baud_rate,
                           uint8_t data_bits,
                           uint8_t parity_bits,
                           uint8_t stop_bits,
                           mbrtu_timing_t *timing)
{
    uint32_t bits_per_character;

    if (timing == NULL) {
        return MBRTU_ERROR_ARGUMENT;
    }

    if (baud_rate == 0u ||
        data_bits < 5u || data_bits > 9u ||
        parity_bits > 1u || stop_bits < 1u || stop_bits > 2u) {
        return MBRTU_ERROR_SERIAL_CONFIG;
    }

    bits_per_character = 1u + (uint32_t)data_bits +
                         (uint32_t)parity_bits + (uint32_t)stop_bits;

    timing->character_ticks = ceil_div_u64(
        (uint64_t)bits_per_character * 1000000u,
        (uint64_t)baud_rate * MODBUS_RTU_TICK_US);

    if (baud_rate > 19200u) {
        timing->t1_5_ticks = 15u;
        timing->t3_5_ticks = 35u;
        timing->byte_complete_t1_5_ticks = ceil_div_u64(
            (750u * (uint64_t)baud_rate) +
                ((uint64_t)bits_per_character * 1000000u),
            (uint64_t)baud_rate * MODBUS_RTU_TICK_US);
        timing->byte_complete_t3_5_ticks = ceil_div_u64(
            (1750u * (uint64_t)baud_rate) +
                ((uint64_t)bits_per_character * 1000000u),
            (uint64_t)baud_rate * MODBUS_RTU_TICK_US);
    } else {
        /*
         * Character time is bits_per_character / baud_rate seconds. Express
         * the 1.5, 2.5, 3.5, and 4.5 character multipliers as tenths and
         * round each 50 us tick count up independently. The 2.5 and 4.5
         * values are receive-complete callback deadlines: silent interval
         * plus the duration of the newly received character.
         */
        timing->t1_5_ticks = ceil_div_u64(
            15u * (uint64_t)bits_per_character * 1000000u,
            10u * (uint64_t)baud_rate * MODBUS_RTU_TICK_US);
        timing->t3_5_ticks = ceil_div_u64(
            35u * (uint64_t)bits_per_character * 1000000u,
            10u * (uint64_t)baud_rate * MODBUS_RTU_TICK_US);
        timing->byte_complete_t1_5_ticks = ceil_div_u64(
            25u * (uint64_t)bits_per_character * 1000000u,
            10u * (uint64_t)baud_rate * MODBUS_RTU_TICK_US);
        timing->byte_complete_t3_5_ticks = ceil_div_u64(
            45u * (uint64_t)bits_per_character * 1000000u,
            10u * (uint64_t)baud_rate * MODBUS_RTU_TICK_US);
    }

    if (timing->character_ticks == 0u ||
        timing->t1_5_ticks == 0u || timing->t3_5_ticks == 0u ||
        timing->byte_complete_t1_5_ticks == 0u ||
        timing->byte_complete_t3_5_ticks == 0u ||
        timing->t1_5_ticks >= timing->t3_5_ticks ||
        timing->byte_complete_t1_5_ticks >=
            timing->byte_complete_t3_5_ticks) {
        return MBRTU_ERROR_SERIAL_CONFIG;
    }

    return 0;
}

static void reset_active_frame(mbrtu_context_t *ctx)
{
    ctx->gap_ticks = 0u;
    ctx->active_length = 0u;
    ctx->frame_active = 0u;
    ctx->active_invalid_gap = 0u;
    ctx->active_overflow = 0u;
}

static void complete_active_frame_isr(mbrtu_context_t *ctx)
{
    if (ctx->frame_active == 0u) {
        return;
    }

    ++ctx->detected_frame_boundaries;

    if (ctx->pending_ready == 0u) {
        /* Publish metadata first and the ready flag last. */
        ctx->pending_buffer_index = ctx->active_buffer_index;
        ctx->pending_length = ctx->active_length;
        ctx->pending_invalid_gap = ctx->active_invalid_gap;
        ctx->pending_overflow = ctx->active_overflow;
        ctx->active_buffer_index ^= 1u;
        ctx->pending_ready = 1u;
        ++ctx->queued_frames;
    } else {
        ++ctx->overrun_frames;
    }

    reset_active_frame(ctx);
}

int mbrtu_init(mbrtu_context_t *ctx, const mbrtu_config_t *config)
{
    mbrtu_timing_t timing;
    int timing_result;

    if (ctx == NULL || config == NULL ||
        config->receive_buffer_a == NULL ||
        config->receive_buffer_b == NULL ||
        config->response_buffer == NULL ||
        config->transmit == NULL ||
        config->receive_buffer_a == config->receive_buffer_b ||
        config->receive_buffer_a == config->response_buffer ||
        config->receive_buffer_b == config->response_buffer ||
        config->receive_buffer_capacity < MODBUS_RTU_ADU_MIN_SIZE ||
        config->receive_buffer_capacity > MODBUS_RTU_ADU_MAX_SIZE ||
        config->response_buffer_capacity <
            1u + 2u + MODBUS_RTU_CRC_SIZE) {
        return MBRTU_ERROR_ARGUMENT;
    }

    if (config->slave_address < MODBUS_RTU_ADDRESS_MIN ||
        config->slave_address > MODBUS_RTU_ADDRESS_MAX) {
        return MBRTU_ERROR_SLAVE_ADDRESS;
    }

    timing_result = mbrtu_calculate_timing(config->baud_rate,
                                            config->data_bits,
                                            config->parity_bits,
                                            config->stop_bits,
                                            &timing);
    if (timing_result != 0) {
        return timing_result;
    }

    ctx->slave_address = config->slave_address;
    ctx->receive_buffers[0] = config->receive_buffer_a;
    ctx->receive_buffers[1] = config->receive_buffer_b;
    ctx->receive_buffer_capacity = config->receive_buffer_capacity;
    ctx->response_buffer = config->response_buffer;
    ctx->response_buffer_capacity = config->response_buffer_capacity;
    ctx->transmit = config->transmit;
    ctx->user_context = config->user_context;
    ctx->timing = timing;

    ctx->active_buffer_index = 0u;
    ctx->pending_buffer_index = 0u;
    ctx->pending_length = 0u;
    ctx->pending_ready = 0u;
    ctx->pending_invalid_gap = 0u;
    ctx->pending_overflow = 0u;
    ctx->initialized = 0u;
    reset_active_frame(ctx);

    ctx->received_bytes = 0u;
    ctx->detected_frame_boundaries = 0u;
    ctx->queued_frames = 0u;
    ctx->invalid_gap_events = 0u;
    ctx->overflow_events = 0u;
    ctx->overrun_frames = 0u;
    ctx->no_response_frames = 0u;
    ctx->responses_sent = 0u;
    ctx->processing_errors = 0u;
    ctx->transmit_errors = 0u;

    ctx->diagnostics = NULL;
    ctx->diagnostics_policy = NULL;
    ctx->diagnostics_policy_context = NULL;

    ctx->initialized = 1u;
    return 0;
}

int mbrtu_init_with_diagnostics(
    mbrtu_context_t *ctx,
    const mbrtu_config_t *config,
    mbrtu_diagnostics_t *diagnostics,
    mbrtu_diagnostics_policy_fn diagnostics_policy,
    void *diagnostics_policy_context)
{
    int result;

    if (diagnostics == NULL) {
        return MBRTU_ERROR_ARGUMENT;
    }

    result = mbrtu_init(ctx, config);
    if (result != 0) {
        return result;
    }

    ctx->diagnostics = diagnostics;
    ctx->diagnostics_policy = diagnostics_policy;
    ctx->diagnostics_policy_context = diagnostics_policy_context;
    return 0;
}

void mbrtu_on_rx_byte_isr(mbrtu_context_t *ctx, uint8_t byte)
{
    uint8_t *active_buffer;

    if (ctx == NULL || ctx->initialized == 0u) {
        return;
    }

    ++ctx->received_bytes;

    if (ctx->frame_active != 0u) {
        if (ctx->gap_ticks >=
            ctx->timing.byte_complete_t3_5_ticks) {
            complete_active_frame_isr(ctx);
        } else if (ctx->gap_ticks >
                       ctx->timing.byte_complete_t1_5_ticks &&
                   ctx->active_invalid_gap == 0u) {
            /* A gap equal to T1.5 is accepted; only a longer gap is invalid. */
            ctx->active_invalid_gap = 1u;
            ++ctx->invalid_gap_events;
        }
    }

    ctx->gap_ticks = 0u;
    ctx->frame_active = 1u;

    if (ctx->active_invalid_gap != 0u || ctx->active_overflow != 0u) {
        /* Keep timing the frame, but avoid useless writes after it is invalid. */
        return;
    }

    if (ctx->active_length >= ctx->receive_buffer_capacity) {
        ctx->active_overflow = 1u;
        ++ctx->overflow_events;
        return;
    }

    active_buffer = ctx->receive_buffers[ctx->active_buffer_index];
    active_buffer[ctx->active_length] = byte;
    ++ctx->active_length;
}

void mbrtu_on_50us_tick_isr(mbrtu_context_t *ctx)
{
    if (ctx == NULL || ctx->initialized == 0u ||
        ctx->frame_active == 0u) {
        return;
    }

    if (ctx->gap_ticks < ctx->timing.byte_complete_t3_5_ticks) {
        ++ctx->gap_ticks;
    }

    if (ctx->gap_ticks >= ctx->timing.byte_complete_t3_5_ticks) {
        complete_active_frame_isr(ctx);
    }
}

int mbrtu_poll(mbrtu_context_t *ctx)
{
    const uint8_t *request;
    size_t request_length;
    size_t response_length = 0u;
    uint8_t invalid_gap;
    uint8_t overflow;
    int process_result;

    if (ctx == NULL || ctx->initialized == 0u) {
        return MBRTU_ERROR_ARGUMENT;
    }

    if (ctx->pending_ready == 0u) {
        return MBRTU_POLL_IDLE;
    }

    request = ctx->receive_buffers[ctx->pending_buffer_index];
    request_length = ctx->pending_length;
    invalid_gap = ctx->pending_invalid_gap;
    overflow = ctx->pending_overflow;

    if (invalid_gap != 0u || overflow != 0u) {
        uint8_t broadcast = (uint8_t)(
            request_length > 0u &&
            request[0] == MODBUS_RTU_BROADCAST_ADDRESS);

        if (overflow != 0u) {
            mbrtu_diagnostics_note_character_overrun(
                ctx->diagnostics,
                broadcast,
                (uint8_t)(request_length > 0u &&
                          request[0] == ctx->slave_address));
        } else {
            mbrtu_diagnostics_note_communication_error(ctx->diagnostics,
                                                       broadcast,
                                                       0u);
        }
        ctx->pending_ready = 0u;
        return MBRTU_POLL_FRAME_DROPPED;
    }

    process_result = mbrtu_process_adu_with_diagnostics(
        ctx->slave_address,
        request,
        request_length,
        ctx->response_buffer,
        ctx->response_buffer_capacity,
        &response_length,
        ctx->diagnostics,
        ctx->diagnostics_policy,
        ctx->diagnostics_policy_context);

    /* The request buffer is no longer needed after complete-frame processing. */
    ctx->pending_ready = 0u;

    if (process_result == MBRTU_RESPONSE_READY) {
        if (ctx->transmit(ctx->user_context,
                          ctx->response_buffer,
                          response_length) != 0) {
            ++ctx->transmit_errors;
            return MBRTU_ERROR_TRANSMIT;
        }
        ++ctx->responses_sent;
        return MBRTU_POLL_RESPONSE_SENT;
    }

    if (process_result == MBRTU_NO_RESPONSE) {
        ++ctx->no_response_frames;
        return MBRTU_POLL_NO_RESPONSE;
    }

    ++ctx->processing_errors;
    return process_result;
}

void mbrtu_get_stats(const mbrtu_context_t *ctx, mbrtu_stats_t *stats)
{
    if (ctx == NULL || stats == NULL) {
        return;
    }

    stats->received_bytes = ctx->received_bytes;
    stats->detected_frame_boundaries = ctx->detected_frame_boundaries;
    stats->queued_frames = ctx->queued_frames;
    stats->invalid_gap_events = ctx->invalid_gap_events;
    stats->overflow_events = ctx->overflow_events;
    stats->overrun_frames = ctx->overrun_frames;
    stats->no_response_frames = ctx->no_response_frames;
    stats->responses_sent = ctx->responses_sent;
    stats->processing_errors = ctx->processing_errors;
    stats->transmit_errors = ctx->transmit_errors;
}
