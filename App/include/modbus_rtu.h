#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "modbus_pdu.h"
#include "modbus_rtu_diagnostics.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_RTU_ADDRESS_MIN 1u
#define MODBUS_RTU_ADDRESS_MAX 247u
#define MODBUS_RTU_BROADCAST_ADDRESS 0u
#define MODBUS_RTU_CRC_SIZE 2u
#define MODBUS_RTU_ADU_MIN_SIZE 4u
#define MODBUS_RTU_ADU_MAX_SIZE (1u + MODBUS_PDU_MAX_SIZE + MODBUS_RTU_CRC_SIZE)
#define MODBUS_RTU_TICK_US 50u

#define MBRTU_NO_RESPONSE 0
#define MBRTU_RESPONSE_READY 1
#define MBRTU_ERROR_ARGUMENT (-1)
#define MBRTU_ERROR_SLAVE_ADDRESS (-2)
#define MBRTU_ERROR_RESPONSE_CAPACITY (-3)
#define MBRTU_ERROR_PDU (-4)
#define MBRTU_ERROR_SERIAL_CONFIG (-5)
#define MBRTU_ERROR_TRANSMIT (-6)

/* mbrtu_poll() results. */
#define MBRTU_POLL_IDLE 0
#define MBRTU_POLL_RESPONSE_SENT 1
#define MBRTU_POLL_NO_RESPONSE 2
#define MBRTU_POLL_FRAME_DROPPED 3

/**
 * Validate and process exactly one complete Modbus RTU Application Data Unit.
 *
 * request_adu contains: slave address, PDU, CRC low byte, CRC high byte.
 * request_adu and response_adu must not overlap. response_adu receives the
 * same slave address, the shared PDU response, and
 * a newly calculated CRC in low-byte/high-byte wire order.
 *
 * Frames with an invalid CRC, a different slave address, or an invalid RTU
 * length are silently ignored. Address zero is treated as a broadcast:
 * supported write requests are applied without a response, while broadcast
 * reads and unsupported functions are ignored.
 *
 * @return MBRTU_RESPONSE_READY when response_adu is ready to transmit,
 *         MBRTU_NO_RESPONSE when the frame is intentionally ignored or is a
 *         broadcast, or one of the MBRTU_ERROR_* values.
 */
int mbrtu_process_adu(uint8_t slave_address,
                      const uint8_t *request_adu,
                      size_t request_adu_len,
                      uint8_t *response_adu,
                      size_t response_capacity,
                      size_t *response_adu_len);

/**
 * Extended complete-frame RTU processing with serial-only diagnostics.
 *
 * The original mbrtu_process_adu() remains unchanged and processes ordinary
 * function codes only. Pass a caller-owned diagnostics state here to enable
 * FC07, FC08, FC0B, and FC0C exclusively on the RTU path.
 */
int mbrtu_process_adu_with_diagnostics(
    uint8_t slave_address,
    const uint8_t *request_adu,
    size_t request_adu_len,
    uint8_t *response_adu,
    size_t response_capacity,
    size_t *response_adu_len,
    mbrtu_diagnostics_t *diagnostics,
    mbrtu_diagnostics_policy_fn diagnostics_policy,
    void *diagnostics_policy_context);

typedef struct {
    uint32_t character_ticks;
    uint32_t t1_5_ticks;
    uint32_t t3_5_ticks;
    uint32_t byte_complete_t1_5_ticks;
    uint32_t byte_complete_t3_5_ticks;
} mbrtu_timing_t;

/**
 * Calculate character, T1.5, and T3.5 timing in fixed 50 microsecond ticks.
 *
 * parity_bits is 0 or 1. stop_bits is 1 or 2. For baud rates above 19200,
 * the Modbus-recommended fixed 750 us and 1750 us silent intervals are used.
 * Lower baud rates use the configured complete character length and ceiling
 * division so a required interval is never declared complete early.
 *
 * The byte_complete_* deadlines account for one additional character time
 * because mbrtu_on_rx_byte_isr() is called when a received byte completes,
 * not when its start bit first appears on the wire.
 */
int mbrtu_calculate_timing(uint32_t baud_rate,
                           uint8_t data_bits,
                           uint8_t parity_bits,
                           uint8_t stop_bits,
                           mbrtu_timing_t *timing);

typedef int (*mbrtu_transmit_callback_t)(void *user_context,
                                         const uint8_t *data,
                                         size_t length);

typedef struct {
    uint8_t slave_address;
    uint32_t baud_rate;
    uint8_t data_bits;
    uint8_t parity_bits;
    uint8_t stop_bits;

    uint8_t *receive_buffer_a;
    uint8_t *receive_buffer_b;
    size_t receive_buffer_capacity;

    uint8_t *response_buffer;
    size_t response_buffer_capacity;

    mbrtu_transmit_callback_t transmit;
    void *user_context;
} mbrtu_config_t;

typedef struct {
    uint32_t received_bytes;
    uint32_t detected_frame_boundaries;
    uint32_t queued_frames;
    uint32_t invalid_gap_events;
    uint32_t overflow_events;
    uint32_t overrun_frames;
    uint32_t no_response_frames;
    uint32_t responses_sent;
    uint32_t processing_errors;
    uint32_t transmit_errors;
} mbrtu_stats_t;

/**
 * Portable bare-metal RTU server context.
 *
 * The receive/tick ISR entry points own the active receive buffer. mbrtu_poll
 * owns a separately published completed buffer. The two caller-provided
 * receive buffers and response buffer must not overlap and must remain valid
 * for the lifetime of the context.
 */
typedef struct {
    uint8_t slave_address;
    uint8_t *receive_buffers[2];
    size_t receive_buffer_capacity;
    uint8_t *response_buffer;
    size_t response_buffer_capacity;
    mbrtu_transmit_callback_t transmit;
    void *user_context;
    mbrtu_timing_t timing;

    volatile uint32_t gap_ticks;
    volatile size_t active_length;
    volatile size_t pending_length;
    volatile uint8_t active_buffer_index;
    volatile uint8_t pending_buffer_index;
    volatile uint8_t frame_active;
    volatile uint8_t active_invalid_gap;
    volatile uint8_t active_overflow;
    volatile uint8_t pending_ready;
    volatile uint8_t pending_invalid_gap;
    volatile uint8_t pending_overflow;
    volatile uint8_t initialized;

    volatile uint32_t received_bytes;
    volatile uint32_t detected_frame_boundaries;
    volatile uint32_t queued_frames;
    volatile uint32_t invalid_gap_events;
    volatile uint32_t overflow_events;
    volatile uint32_t overrun_frames;
    volatile uint32_t no_response_frames;
    volatile uint32_t responses_sent;
    volatile uint32_t processing_errors;
    volatile uint32_t transmit_errors;

    mbrtu_diagnostics_t *diagnostics;
    mbrtu_diagnostics_policy_fn diagnostics_policy;
    void *diagnostics_policy_context;
} mbrtu_context_t;

/** Initialize the portable byte/timing RTU server layer. */
int mbrtu_init(mbrtu_context_t *ctx, const mbrtu_config_t *config);

/**
 * Initialize the RTU server layer and attach serial-only diagnostics state.
 *
 * diagnostics must remain valid for the context lifetime. A NULL policy denies
 * every state-changing FC08 subfunction while still allowing read-only
 * diagnostics.
 */
int mbrtu_init_with_diagnostics(
    mbrtu_context_t *ctx,
    const mbrtu_config_t *config,
    mbrtu_diagnostics_t *diagnostics,
    mbrtu_diagnostics_policy_fn diagnostics_policy,
    void *diagnostics_policy_context);

/**
 * Store one completed UART byte. Call from the receive-complete ISR/callback.
 *
 * The callback is assumed to occur after the complete character, including
 * its stop bit, has been received.
 */
void mbrtu_on_rx_byte_isr(mbrtu_context_t *ctx, uint8_t byte);

/** Advance RTU timing by exactly 50 microseconds. Call from the timing ISR. */
void mbrtu_on_50us_tick_isr(mbrtu_context_t *ctx);

/**
 * Process at most one completed frame from the bare-metal main loop.
 *
 * The transmit callback is invoked only from this function, never from an
 * ISR. It must return 0 on success.
 */
int mbrtu_poll(mbrtu_context_t *ctx);

/** Obtain a best-effort diagnostic snapshot without disabling interrupts. */
void mbrtu_get_stats(const mbrtu_context_t *ctx, mbrtu_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
