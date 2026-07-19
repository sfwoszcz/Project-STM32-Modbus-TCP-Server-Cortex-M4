#include "app.h"

#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*
 * STM32F767 Modbus RTU master hardware-validation harness.
 *
 * The proven USART3 byte receiver and 50 us T1.5/T3.5 frame detector are kept
 * board-specific. Request ownership, response deadlines, retries, transport
 * events, response validation, and terminal transaction state are delegated to
 * the portable mbrtum_transaction_* engine.
 *
 * Exactly one test is active at a time. FC05, FC06, FC0F, and FC10 perform a
 * validated write acknowledgement followed by an FC01 or FC03 read-back.
 */

#define MODBUS_MASTER_SLAVE_ADDRESS             1u
#define MODBUS_MASTER_BAUD_RATE                 115200u
#define MODBUS_MASTER_DATA_BITS                 8u
#define MODBUS_MASTER_PARITY_BITS               0u
#define MODBUS_MASTER_STOP_BITS                 1u
#define MODBUS_MASTER_RESPONSE_TIMEOUT_MS       1000u
#define MODBUS_MASTER_RETRY_DELAY_MS            100u
#define MODBUS_MASTER_MAX_RETRIES               2u
#define MODBUS_MASTER_REQUEST_PERIOD_MS         1000u
#define MODBUS_MASTER_FIRST_REQUEST_DELAY_MS    500u
#define MODBUS_MASTER_UART_TIMEOUT_MS           100u

#define MODBUS_MASTER_RX_BUFFER_CAPACITY MODBUS_RTU_ADU_MAX_SIZE
#define MODBUS_MASTER_PACKED_COIL_BYTES \
    ((MODBUS_MASTER_TEST_QUANTITY + 7u) / 8u)
#define MODBUS_MASTER_NO_MISMATCH UINT16_MAX

typedef struct {
    uint8_t buffers[2][MODBUS_MASTER_RX_BUFFER_CAPACITY];
    mbrtu_timing_t timing;

    volatile uint32_t gap_ticks;
    volatile uint16_t active_length;
    volatile uint16_t pending_length;
    volatile uint8_t active_buffer_index;
    volatile uint8_t pending_buffer_index;
    volatile uint8_t frame_active;
    volatile uint8_t active_invalid_gap;
    volatile uint8_t active_overflow;
    volatile uint8_t pending_ready;
    volatile uint8_t pending_invalid_gap;
    volatile uint8_t pending_overflow;
} modbus_master_rx_t;

static modbus_master_rx_t modbus_master_rx;
static mbrtum_request_t modbus_master_request;
static uint8_t modbus_master_request_adu[MODBUS_RTU_ADU_MAX_SIZE];
static size_t modbus_master_request_adu_length;
static uint8_t modbus_master_uart_rx_byte;
static uint32_t modbus_master_next_cycle_ms;
static uint32_t modbus_master_tx_complete_tick;
static uint8_t modbus_master_expected_coils[MODBUS_MASTER_TEST_QUANTITY];
static uint16_t modbus_master_expected_registers[MODBUS_MASTER_TEST_QUANTITY];
static uint8_t modbus_master_packed_coils[MODBUS_MASTER_PACKED_COIL_BYTES];
static volatile uint8_t modbus_master_initialized;
static volatile uint8_t modbus_master_force_request;
static volatile uint8_t modbus_master_uart_error_pending;
static volatile uint8_t modbus_master_tx_complete_pending;
static volatile uint8_t modbus_master_receive_window_open;
static uint8_t modbus_master_cycle_active;

mbrtum_transaction_t modbus_master_transaction;

volatile uint32_t systick_count = 0u;
volatile modbus_master_test_t modbus_master_selected_test =
    MODBUS_MASTER_ACTIVE_TEST;
volatile modbus_master_test_t modbus_master_current_test =
    MODBUS_MASTER_ACTIVE_TEST;
volatile modbus_master_test_step_t modbus_master_test_step =
    MODBUS_MASTER_TEST_STEP_IDLE;
volatile modbus_master_result_t modbus_master_last_result =
    MODBUS_MASTER_RESULT_NONE;
volatile mbrtum_transaction_state_t modbus_master_transaction_state =
    MBRTUM_TXN_STATE_IDLE;
volatile mbrtum_transaction_result_t modbus_master_transaction_result =
    MBRTUM_TXN_RESULT_NONE;
volatile int modbus_master_last_protocol_result = MBRTUM_OK;
volatile uint8_t modbus_master_last_exception_code = 0u;
volatile uint16_t modbus_master_last_response_length = 0u;
volatile uint8_t modbus_master_current_function = 0u;
volatile uint8_t modbus_master_auto_poll_enabled = 1u;

volatile uint32_t modbus_master_test_cycles_started = 0u;
volatile uint32_t modbus_master_test_cycles_completed = 0u;
volatile uint32_t modbus_master_test_cycles_passed = 0u;
volatile uint32_t modbus_master_test_cycles_failed = 0u;
volatile uint32_t modbus_master_transactions_started = 0u;
volatile uint32_t modbus_master_requests_sent = 0u;
volatile uint32_t modbus_master_valid_responses = 0u;
volatile uint32_t modbus_master_exception_responses = 0u;
volatile uint32_t modbus_master_timeouts = 0u;
volatile uint32_t modbus_master_retries = 0u;
volatile uint32_t modbus_master_transmit_errors = 0u;
volatile uint32_t modbus_master_response_errors = 0u;
volatile uint32_t modbus_master_unrelated_responses = 0u;
volatile uint32_t modbus_master_late_responses = 0u;
volatile uint32_t modbus_master_cancelled_transactions = 0u;
volatile uint32_t modbus_master_build_errors = 0u;
volatile uint32_t modbus_master_decode_errors = 0u;
volatile uint32_t modbus_master_verification_passes = 0u;
volatile uint32_t modbus_master_verification_failures = 0u;

volatile uint32_t modbus_master_received_bytes = 0u;
volatile uint32_t modbus_master_detected_frames = 0u;
volatile uint32_t modbus_master_invalid_gap_frames = 0u;
volatile uint32_t modbus_master_overflow_frames = 0u;
volatile uint32_t modbus_master_overrun_frames = 0u;
volatile uint32_t modbus_master_unexpected_bytes = 0u;
volatile uint32_t modbus_uart_error_count = 0u;
volatile uint32_t modbus_uart_recovery_count = 0u;

uint8_t modbus_master_coils[MODBUS_MASTER_TEST_QUANTITY];
uint8_t modbus_master_discrete_inputs[MODBUS_MASTER_TEST_QUANTITY];
uint16_t modbus_master_holding_registers[MODBUS_MASTER_TEST_QUANTITY];
uint16_t modbus_master_input_registers[MODBUS_MASTER_TEST_QUANTITY];

volatile uint8_t modbus_master_single_coil_write_value = 1u;
volatile uint16_t modbus_master_single_register_write_value = 0x1234u;
volatile uint8_t modbus_master_multiple_coil_write_values[MODBUS_MASTER_TEST_QUANTITY] = {
    1u, 0u, 1u, 0u, 1u, 0u, 1u, 0u, 1u, 0u
};
volatile uint16_t modbus_master_multiple_register_write_values[
    MODBUS_MASTER_TEST_QUANTITY] = {
        0x1000u, 0x1001u, 0x1002u, 0x1003u, 0x1004u,
        0x1005u, 0x1006u, 0x1007u, 0x1008u, 0x1009u
    };

volatile uint16_t modbus_master_verification_mismatch_index =
    MODBUS_MASTER_NO_MISMATCH;
volatile uint16_t modbus_master_verification_expected = 0u;
volatile uint16_t modbus_master_verification_actual = 0u;

static int modbus_master_time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static int modbus_master_test_is_valid(modbus_master_test_t test)
{
    return test == MODBUS_MASTER_TEST_FC01 ||
           test == MODBUS_MASTER_TEST_FC02 ||
           test == MODBUS_MASTER_TEST_FC03 ||
           test == MODBUS_MASTER_TEST_FC04 ||
           test == MODBUS_MASTER_TEST_FC05 ||
           test == MODBUS_MASTER_TEST_FC06 ||
           test == MODBUS_MASTER_TEST_FC0F ||
           test == MODBUS_MASTER_TEST_FC10;
}

static int modbus_master_test_is_write(modbus_master_test_t test)
{
    return test == MODBUS_MASTER_TEST_FC05 ||
           test == MODBUS_MASTER_TEST_FC06 ||
           test == MODBUS_MASTER_TEST_FC0F ||
           test == MODBUS_MASTER_TEST_FC10;
}

static uint32_t modbus_master_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void modbus_master_exit_critical(uint32_t primask)
{
    if ((primask & 1u) == 0u) {
        __enable_irq();
    }
}

static void modbus_master_rx_reset_active_isr(void)
{
    modbus_master_rx.gap_ticks = 0u;
    modbus_master_rx.active_length = 0u;
    modbus_master_rx.frame_active = 0u;
    modbus_master_rx.active_invalid_gap = 0u;
    modbus_master_rx.active_overflow = 0u;
}

static void modbus_master_rx_reset_all(void)
{
    uint32_t primask = modbus_master_enter_critical();

    modbus_master_rx.active_buffer_index = 0u;
    modbus_master_rx.pending_buffer_index = 0u;
    modbus_master_rx.pending_length = 0u;
    modbus_master_rx.pending_ready = 0u;
    modbus_master_rx.pending_invalid_gap = 0u;
    modbus_master_rx.pending_overflow = 0u;
    modbus_master_rx_reset_active_isr();

    modbus_master_exit_critical(primask);
}

static void modbus_master_rx_complete_frame_isr(void)
{
    if (modbus_master_rx.frame_active == 0u) {
        return;
    }

    ++modbus_master_detected_frames;

    if (modbus_master_rx.pending_ready == 0u) {
        modbus_master_rx.pending_buffer_index =
            modbus_master_rx.active_buffer_index;
        modbus_master_rx.pending_length = modbus_master_rx.active_length;
        modbus_master_rx.pending_invalid_gap =
            modbus_master_rx.active_invalid_gap;
        modbus_master_rx.pending_overflow = modbus_master_rx.active_overflow;
        modbus_master_rx.active_buffer_index ^= 1u;
        modbus_master_rx.pending_ready = 1u;
    } else {
        ++modbus_master_overrun_frames;
    }

    modbus_master_rx_reset_active_isr();
}

static void modbus_master_on_rx_byte_isr(uint8_t byte)
{
    uint8_t *active_buffer;

    if (modbus_master_initialized == 0u ||
        modbus_master_receive_window_open == 0u) {
        ++modbus_master_unexpected_bytes;
        return;
    }

    ++modbus_master_received_bytes;

    if (modbus_master_rx.frame_active != 0u) {
        if (modbus_master_rx.gap_ticks >=
            modbus_master_rx.timing.byte_complete_t3_5_ticks) {
            modbus_master_rx_complete_frame_isr();
        } else if (modbus_master_rx.gap_ticks >
                       modbus_master_rx.timing.byte_complete_t1_5_ticks &&
                   modbus_master_rx.active_invalid_gap == 0u) {
            modbus_master_rx.active_invalid_gap = 1u;
            ++modbus_master_invalid_gap_frames;
        }
    }

    modbus_master_rx.gap_ticks = 0u;
    modbus_master_rx.frame_active = 1u;

    if (modbus_master_rx.active_invalid_gap != 0u ||
        modbus_master_rx.active_overflow != 0u) {
        return;
    }

    if (modbus_master_rx.active_length >= MODBUS_MASTER_RX_BUFFER_CAPACITY) {
        modbus_master_rx.active_overflow = 1u;
        ++modbus_master_overflow_frames;
        return;
    }

    active_buffer =
        modbus_master_rx.buffers[modbus_master_rx.active_buffer_index];
    active_buffer[modbus_master_rx.active_length] = byte;
    ++modbus_master_rx.active_length;
}

static void modbus_master_on_50us_tick_isr(void)
{
    if (modbus_master_initialized == 0u ||
        modbus_master_receive_window_open == 0u ||
        modbus_master_rx.frame_active == 0u) {
        return;
    }

    if (modbus_master_rx.gap_ticks < modbus_master_rx.timing.t3_5_ticks) {
        ++modbus_master_rx.gap_ticks;
    }

    if (modbus_master_rx.gap_ticks >= modbus_master_rx.timing.t3_5_ticks) {
        modbus_master_rx_complete_frame_isr();
    }
}

static void modbus_master_ensure_receive_armed(void)
{
    if (huart3.RxState == HAL_UART_STATE_READY) {
        if (HAL_UART_Receive_IT(&huart3, &modbus_master_uart_rx_byte, 1u) ==
            HAL_OK) {
            ++modbus_uart_recovery_count;
        } else {
            ++modbus_uart_error_count;
            modbus_master_uart_error_pending = 1u;
        }
    }
}

static void modbus_master_sync_watch_state(void)
{
    const mbrtum_transaction_diagnostics_t *diagnostics =
        &modbus_master_transaction.diagnostics;

    modbus_master_transaction_state = modbus_master_transaction.state;
    modbus_master_transaction_result = modbus_master_transaction.result;
    modbus_master_last_exception_code = modbus_master_transaction.exception_code;

    if (modbus_master_cycle_active != 0u ||
        (modbus_master_last_result != MODBUS_MASTER_RESULT_BUILD_ERROR &&
         modbus_master_last_result != MODBUS_MASTER_RESULT_START_ERROR &&
         modbus_master_last_result != MODBUS_MASTER_RESULT_INVALID_TEST)) {
        modbus_master_last_protocol_result =
            modbus_master_transaction.protocol_result;
    }

    modbus_master_transactions_started = diagnostics->transactions_started;
    modbus_master_requests_sent = diagnostics->transmission_attempts;
    modbus_master_valid_responses = diagnostics->successful_transactions;
    modbus_master_exception_responses = diagnostics->exception_responses;
    modbus_master_timeouts = diagnostics->timeouts;
    modbus_master_retries = diagnostics->retries_performed;
    modbus_master_transmit_errors = diagnostics->transport_errors;
    modbus_master_response_errors = diagnostics->malformed_responses;
    modbus_master_unrelated_responses = diagnostics->unrelated_responses;
    modbus_master_late_responses = diagnostics->late_responses;
    modbus_master_cancelled_transactions =
        diagnostics->cancelled_transactions;

    if (modbus_master_transaction.state != MBRTUM_TXN_STATE_WAITING_RESPONSE &&
        !(modbus_master_transaction.state == MBRTUM_TXN_STATE_TRANSMITTING &&
          modbus_master_tx_complete_pending != 0u)) {
        modbus_master_receive_window_open = 0u;
        modbus_master_rx_reset_all();
    }
}

static int modbus_master_transport_transmit(void *context,
                                            const uint8_t *adu,
                                            size_t adu_length)
{
    HAL_StatusTypeDef transmit_result;

    (void)context;

    if (adu == NULL || adu_length == 0u ||
        adu_length > (size_t)UINT16_MAX) {
        return -1;
    }

    modbus_master_rx_reset_all();
    modbus_master_receive_window_open = 0u;
    modbus_master_tx_complete_pending = 0u;
    modbus_master_current_function = adu[1];

    transmit_result = HAL_UART_Transmit(&huart3,
                                        (uint8_t *)adu,
                                        (uint16_t)adu_length,
                                        MODBUS_MASTER_UART_TIMEOUT_MS);
    if (transmit_result != HAL_OK) {
        return -1;
    }

    modbus_master_tx_complete_tick = HAL_GetTick();
    modbus_master_tx_complete_pending = 1u;
    modbus_master_receive_window_open = 1u;
    return MBRTUM_TXN_TRANSMIT_ACCEPTED;
}

static void modbus_master_handle_tx_complete(void)
{
    uint8_t pending;
    uint32_t completed_at;
    uint32_t primask;
    int result;

    primask = modbus_master_enter_critical();
    pending = modbus_master_tx_complete_pending;
    completed_at = modbus_master_tx_complete_tick;
    modbus_master_tx_complete_pending = 0u;
    modbus_master_exit_critical(primask);

    if (pending == 0u) {
        return;
    }

    result = mbrtum_transaction_on_tx_complete(&modbus_master_transaction,
                                                completed_at);
    if (result != MBRTUM_TXN_OK) {
        modbus_master_last_protocol_result = result;
        modbus_master_receive_window_open = 0u;
    }
}

static int modbus_master_build_request(void)
{
    uint16_t index;

    memset(&modbus_master_request, 0, sizeof(modbus_master_request));
    memset(modbus_master_request_adu, 0, sizeof(modbus_master_request_adu));
    modbus_master_request_adu_length = 0u;

    if (modbus_master_test_step == MODBUS_MASTER_TEST_STEP_VERIFY_READBACK) {
        if (modbus_master_current_test == MODBUS_MASTER_TEST_FC05 ||
            modbus_master_current_test == MODBUS_MASTER_TEST_FC0F) {
            return mbrtum_build_read_bits_request(
                MODBUS_MASTER_SLAVE_ADDRESS,
                MBRTUM_FC_READ_COILS,
                MODBUS_MASTER_TEST_START_ADDRESS,
                MODBUS_MASTER_TEST_QUANTITY,
                &modbus_master_request,
                modbus_master_request_adu,
                sizeof(modbus_master_request_adu),
                &modbus_master_request_adu_length);
        }

        return mbrtum_build_read_registers_request(
            MODBUS_MASTER_SLAVE_ADDRESS,
            MBRTUM_FC_READ_HOLDING_REGISTERS,
            MODBUS_MASTER_TEST_START_ADDRESS,
            MODBUS_MASTER_TEST_QUANTITY,
            &modbus_master_request,
            modbus_master_request_adu,
            sizeof(modbus_master_request_adu),
            &modbus_master_request_adu_length);
    }

    switch (modbus_master_current_test) {
    case MODBUS_MASTER_TEST_FC01:
    case MODBUS_MASTER_TEST_FC02:
        return mbrtum_build_read_bits_request(
            MODBUS_MASTER_SLAVE_ADDRESS,
            (uint8_t)modbus_master_current_test,
            MODBUS_MASTER_TEST_START_ADDRESS,
            MODBUS_MASTER_TEST_QUANTITY,
            &modbus_master_request,
            modbus_master_request_adu,
            sizeof(modbus_master_request_adu),
            &modbus_master_request_adu_length);

    case MODBUS_MASTER_TEST_FC03:
    case MODBUS_MASTER_TEST_FC04:
        return mbrtum_build_read_registers_request(
            MODBUS_MASTER_SLAVE_ADDRESS,
            (uint8_t)modbus_master_current_test,
            MODBUS_MASTER_TEST_START_ADDRESS,
            MODBUS_MASTER_TEST_QUANTITY,
            &modbus_master_request,
            modbus_master_request_adu,
            sizeof(modbus_master_request_adu),
            &modbus_master_request_adu_length);

    case MODBUS_MASTER_TEST_FC05:
        modbus_master_expected_coils[0] =
            modbus_master_single_coil_write_value != 0u ? 1u : 0u;
        return mbrtum_build_write_single_coil_request(
            MODBUS_MASTER_SLAVE_ADDRESS,
            MODBUS_MASTER_TEST_START_ADDRESS,
            modbus_master_expected_coils[0],
            &modbus_master_request,
            modbus_master_request_adu,
            sizeof(modbus_master_request_adu),
            &modbus_master_request_adu_length);

    case MODBUS_MASTER_TEST_FC06:
        modbus_master_expected_registers[0] =
            modbus_master_single_register_write_value;
        return mbrtum_build_write_single_register_request(
            MODBUS_MASTER_SLAVE_ADDRESS,
            MODBUS_MASTER_TEST_START_ADDRESS,
            modbus_master_expected_registers[0],
            &modbus_master_request,
            modbus_master_request_adu,
            sizeof(modbus_master_request_adu),
            &modbus_master_request_adu_length);

    case MODBUS_MASTER_TEST_FC0F:
        memset(modbus_master_packed_coils,
               0,
               sizeof(modbus_master_packed_coils));
        for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
            uint8_t value =
                modbus_master_multiple_coil_write_values[index] != 0u ? 1u : 0u;
            modbus_master_expected_coils[index] = value;
            if (value != 0u) {
                modbus_master_packed_coils[index / 8u] |=
                    (uint8_t)(1u << (index % 8u));
            }
        }
        return mbrtum_build_write_multiple_coils_request(
            MODBUS_MASTER_SLAVE_ADDRESS,
            MODBUS_MASTER_TEST_START_ADDRESS,
            MODBUS_MASTER_TEST_QUANTITY,
            modbus_master_packed_coils,
            &modbus_master_request,
            modbus_master_request_adu,
            sizeof(modbus_master_request_adu),
            &modbus_master_request_adu_length);

    case MODBUS_MASTER_TEST_FC10:
        for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
            modbus_master_expected_registers[index] =
                modbus_master_multiple_register_write_values[index];
        }
        return mbrtum_build_write_multiple_registers_request(
            MODBUS_MASTER_SLAVE_ADDRESS,
            MODBUS_MASTER_TEST_START_ADDRESS,
            MODBUS_MASTER_TEST_QUANTITY,
            modbus_master_expected_registers,
            &modbus_master_request,
            modbus_master_request_adu,
            sizeof(modbus_master_request_adu),
            &modbus_master_request_adu_length);

    default:
        return MBRTUM_ERROR_FUNCTION;
    }
}

static void modbus_master_finish_cycle(modbus_master_result_t result,
                                       uint32_t now)
{
    modbus_master_cycle_active = 0u;
    modbus_master_test_step = MODBUS_MASTER_TEST_STEP_IDLE;
    modbus_master_last_result = result;
    ++modbus_master_test_cycles_completed;

    if (result == MODBUS_MASTER_RESULT_PASS) {
        ++modbus_master_test_cycles_passed;
    } else {
        ++modbus_master_test_cycles_failed;
    }

    modbus_master_next_cycle_ms = now + MODBUS_MASTER_REQUEST_PERIOD_MS;
    modbus_master_receive_window_open = 0u;
    modbus_master_rx_reset_all();
}

static int modbus_master_start_current_step(uint32_t now)
{
    int result;

    result = modbus_master_build_request();
    if (result != MBRTUM_OK) {
        ++modbus_master_build_errors;
        modbus_master_last_protocol_result = result;
        modbus_master_finish_cycle(MODBUS_MASTER_RESULT_BUILD_ERROR, now);
        return result;
    }

    result = mbrtum_transaction_start(&modbus_master_transaction,
                                      &modbus_master_request,
                                      modbus_master_request_adu,
                                      modbus_master_request_adu_length,
                                      now);
    if (result != MBRTUM_TXN_OK) {
        modbus_master_last_protocol_result = result;
        modbus_master_finish_cycle(MODBUS_MASTER_RESULT_START_ERROR, now);
        return result;
    }

    modbus_master_handle_tx_complete();
    modbus_master_sync_watch_state();
    return MBRTUM_TXN_OK;
}

static void modbus_master_start_cycle(uint32_t now)
{
    if (modbus_master_cycle_active != 0u) {
        return;
    }

    if (!modbus_master_test_is_valid(modbus_master_selected_test)) {
        modbus_master_last_protocol_result = MBRTUM_ERROR_FUNCTION;
        modbus_master_current_test = modbus_master_selected_test;
        modbus_master_last_result = MODBUS_MASTER_RESULT_INVALID_TEST;
        ++modbus_master_test_cycles_started;
        ++modbus_master_test_cycles_completed;
        ++modbus_master_test_cycles_failed;
        modbus_master_next_cycle_ms = now + MODBUS_MASTER_REQUEST_PERIOD_MS;
        return;
    }

    modbus_master_current_test = modbus_master_selected_test;
    modbus_master_test_step = MODBUS_MASTER_TEST_STEP_REQUEST;
    modbus_master_last_result = MODBUS_MASTER_RESULT_NONE;
    modbus_master_verification_mismatch_index = MODBUS_MASTER_NO_MISMATCH;
    modbus_master_verification_expected = 0u;
    modbus_master_verification_actual = 0u;
    modbus_master_cycle_active = 1u;
    ++modbus_master_test_cycles_started;

    (void)modbus_master_start_current_step(now);
}

static void modbus_master_process_response_frame(uint32_t now)
{
    const uint8_t *response_adu;
    uint16_t response_length;
    uint8_t invalid_gap;
    uint8_t overflow;
    uint8_t pending_buffer_index;
    uint32_t primask;

    if (modbus_master_rx.pending_ready == 0u) {
        return;
    }

    primask = modbus_master_enter_critical();
    pending_buffer_index = modbus_master_rx.pending_buffer_index;
    response_length = modbus_master_rx.pending_length;
    invalid_gap = modbus_master_rx.pending_invalid_gap;
    overflow = modbus_master_rx.pending_overflow;
    modbus_master_rx.pending_ready = 0u;
    modbus_master_exit_critical(primask);

    response_adu = modbus_master_rx.buffers[pending_buffer_index];
    modbus_master_last_response_length = response_length;

    if (modbus_master_transaction.state != MBRTUM_TXN_STATE_WAITING_RESPONSE) {
        ++modbus_master_unexpected_bytes;
        return;
    }

    if (overflow != 0u || invalid_gap != 0u) {
        (void)mbrtum_transaction_on_response(&modbus_master_transaction,
                                             response_adu,
                                             0u,
                                             now);
        return;
    }

    (void)mbrtum_transaction_on_response(&modbus_master_transaction,
                                         response_adu,
                                         (size_t)response_length,
                                         now);
}

static int modbus_master_decode_read_response(void)
{
    uint16_t index;
    int result;

    switch (modbus_master_transaction.request.function) {
    case MBRTUM_FC_READ_COILS:
        for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
            result = mbrtum_get_bit(&modbus_master_transaction.request,
                                    &modbus_master_transaction.response,
                                    index,
                                    &modbus_master_coils[index]);
            if (result != MBRTUM_OK) {
                return result;
            }
        }
        return MBRTUM_OK;

    case MBRTUM_FC_READ_DISCRETE_INPUTS:
        for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
            result = mbrtum_get_bit(&modbus_master_transaction.request,
                                    &modbus_master_transaction.response,
                                    index,
                                    &modbus_master_discrete_inputs[index]);
            if (result != MBRTUM_OK) {
                return result;
            }
        }
        return MBRTUM_OK;

    case MBRTUM_FC_READ_HOLDING_REGISTERS:
        for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
            result = mbrtum_get_register(&modbus_master_transaction.request,
                                         &modbus_master_transaction.response,
                                         index,
                                         &modbus_master_holding_registers[index]);
            if (result != MBRTUM_OK) {
                return result;
            }
        }
        return MBRTUM_OK;

    case MBRTUM_FC_READ_INPUT_REGISTERS:
        for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
            result = mbrtum_get_register(&modbus_master_transaction.request,
                                         &modbus_master_transaction.response,
                                         index,
                                         &modbus_master_input_registers[index]);
            if (result != MBRTUM_OK) {
                return result;
            }
        }
        return MBRTUM_OK;

    default:
        return MBRTUM_OK;
    }
}

static int modbus_master_verify_readback(void)
{
    uint16_t index;
    uint16_t count;

    modbus_master_verification_mismatch_index = MODBUS_MASTER_NO_MISMATCH;
    modbus_master_verification_expected = 0u;
    modbus_master_verification_actual = 0u;

    if (modbus_master_current_test == MODBUS_MASTER_TEST_FC05) {
        count = 1u;
        for (index = 0u; index < count; ++index) {
            if (modbus_master_coils[index] !=
                modbus_master_expected_coils[index]) {
                modbus_master_verification_mismatch_index = index;
                modbus_master_verification_expected =
                    modbus_master_expected_coils[index];
                modbus_master_verification_actual = modbus_master_coils[index];
                return 0;
            }
        }
        return 1;
    }

    if (modbus_master_current_test == MODBUS_MASTER_TEST_FC0F) {
        count = MODBUS_MASTER_TEST_QUANTITY;
        for (index = 0u; index < count; ++index) {
            if (modbus_master_coils[index] !=
                modbus_master_expected_coils[index]) {
                modbus_master_verification_mismatch_index = index;
                modbus_master_verification_expected =
                    modbus_master_expected_coils[index];
                modbus_master_verification_actual = modbus_master_coils[index];
                return 0;
            }
        }
        return 1;
    }

    if (modbus_master_current_test == MODBUS_MASTER_TEST_FC06) {
        count = 1u;
    } else {
        count = MODBUS_MASTER_TEST_QUANTITY;
    }

    for (index = 0u; index < count; ++index) {
        if (modbus_master_holding_registers[index] !=
            modbus_master_expected_registers[index]) {
            modbus_master_verification_mismatch_index = index;
            modbus_master_verification_expected =
                modbus_master_expected_registers[index];
            modbus_master_verification_actual =
                modbus_master_holding_registers[index];
            return 0;
        }
    }

    return 1;
}

static modbus_master_result_t modbus_master_map_terminal_result(void)
{
    switch (modbus_master_transaction.result) {
    case MBRTUM_TXN_RESULT_MODBUS_EXCEPTION:
        return MODBUS_MASTER_RESULT_EXCEPTION;
    case MBRTUM_TXN_RESULT_TIMEOUT:
        return MODBUS_MASTER_RESULT_TIMEOUT;
    case MBRTUM_TXN_RESULT_TRANSPORT_ERROR:
        return MODBUS_MASTER_RESULT_TRANSPORT_ERROR;
    case MBRTUM_TXN_RESULT_RESPONSE_ERROR:
        return MODBUS_MASTER_RESULT_RESPONSE_ERROR;
    case MBRTUM_TXN_RESULT_CANCELLED:
        return MODBUS_MASTER_RESULT_CANCELLED;
    default:
        return MODBUS_MASTER_RESULT_RESPONSE_ERROR;
    }
}

static void modbus_master_handle_terminal_transaction(uint32_t now)
{
    int decode_result;

    if (modbus_master_cycle_active == 0u) {
        return;
    }

    if (modbus_master_transaction.state == MBRTUM_TXN_STATE_COMPLETE) {
        if (modbus_master_transaction.result ==
            MBRTUM_TXN_RESULT_MODBUS_EXCEPTION) {
            modbus_master_finish_cycle(MODBUS_MASTER_RESULT_EXCEPTION, now);
            return;
        }

        if (modbus_master_transaction.result != MBRTUM_TXN_RESULT_SUCCESS) {
            modbus_master_finish_cycle(MODBUS_MASTER_RESULT_RESPONSE_ERROR,
                                       now);
            return;
        }

        if (modbus_master_test_step == MODBUS_MASTER_TEST_STEP_REQUEST &&
            modbus_master_test_is_write(modbus_master_current_test)) {
            modbus_master_test_step =
                MODBUS_MASTER_TEST_STEP_VERIFY_READBACK;
            (void)modbus_master_start_current_step(now);
            return;
        }

        decode_result = modbus_master_decode_read_response();
        if (decode_result != MBRTUM_OK) {
            ++modbus_master_decode_errors;
            modbus_master_last_protocol_result = decode_result;
            modbus_master_finish_cycle(MODBUS_MASTER_RESULT_DECODE_ERROR, now);
            return;
        }

        if (modbus_master_test_step ==
            MODBUS_MASTER_TEST_STEP_VERIFY_READBACK) {
            if (!modbus_master_verify_readback()) {
                ++modbus_master_verification_failures;
                modbus_master_finish_cycle(MODBUS_MASTER_RESULT_VERIFY_ERROR,
                                           now);
                return;
            }
            ++modbus_master_verification_passes;
        }

        modbus_master_finish_cycle(MODBUS_MASTER_RESULT_PASS, now);
        return;
    }

    if (modbus_master_transaction.state == MBRTUM_TXN_STATE_FAILED ||
        modbus_master_transaction.state == MBRTUM_TXN_STATE_CANCELLED) {
        modbus_master_finish_cycle(modbus_master_map_terminal_result(), now);
    }
}

static void modbus_master_process_uart_error(uint32_t now)
{
    uint32_t primask;

    if (modbus_master_uart_error_pending == 0u) {
        return;
    }

    primask = modbus_master_enter_critical();
    modbus_master_uart_error_pending = 0u;
    modbus_master_exit_critical(primask);

    modbus_master_rx_reset_all();
    modbus_master_receive_window_open =
        modbus_master_transaction.state == MBRTUM_TXN_STATE_WAITING_RESPONSE
            ? 1u
            : 0u;

    if (modbus_master_transaction.state == MBRTUM_TXN_STATE_WAITING_RESPONSE) {
        (void)mbrtum_transaction_on_response(&modbus_master_transaction,
                                             modbus_master_rx.buffers[0],
                                             0u,
                                             now);
    }
}

int modbus_master_init(void)
{
    mbrtum_transaction_config_t transaction_config;
    int timing_result;
    int transaction_result;
    uint16_t index;

    memset(&modbus_master_rx, 0, sizeof(modbus_master_rx));
    memset(&modbus_master_transaction, 0, sizeof(modbus_master_transaction));
    memset(&modbus_master_request, 0, sizeof(modbus_master_request));
    memset(modbus_master_request_adu, 0, sizeof(modbus_master_request_adu));
    memset(modbus_master_coils, 0, sizeof(modbus_master_coils));
    memset(modbus_master_discrete_inputs,
           0,
           sizeof(modbus_master_discrete_inputs));
    memset(modbus_master_holding_registers,
           0,
           sizeof(modbus_master_holding_registers));
    memset(modbus_master_input_registers,
           0,
           sizeof(modbus_master_input_registers));
    memset(modbus_master_expected_coils,
           0,
           sizeof(modbus_master_expected_coils));
    memset(modbus_master_expected_registers,
           0,
           sizeof(modbus_master_expected_registers));

    for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
        modbus_master_multiple_coil_write_values[index] =
            (uint8_t)((index & 1u) == 0u ? 1u : 0u);
        modbus_master_multiple_register_write_values[index] =
            (uint16_t)(0x1000u + index);
    }

    timing_result = mbrtu_calculate_timing(MODBUS_MASTER_BAUD_RATE,
                                           MODBUS_MASTER_DATA_BITS,
                                           MODBUS_MASTER_PARITY_BITS,
                                           MODBUS_MASTER_STOP_BITS,
                                           &modbus_master_rx.timing);
    if (timing_result != 0) {
        modbus_master_last_protocol_result = timing_result;
        modbus_master_last_result = MODBUS_MASTER_RESULT_BUILD_ERROR;
        return timing_result;
    }

    transaction_config.response_timeout_ticks =
        MODBUS_MASTER_RESPONSE_TIMEOUT_MS;
    transaction_config.retry_delay_ticks = MODBUS_MASTER_RETRY_DELAY_MS;
    transaction_config.max_retries = MODBUS_MASTER_MAX_RETRIES;
    transaction_config.retry_transport_errors = 1u;

    transaction_result = mbrtum_transaction_init(
        &modbus_master_transaction,
        &transaction_config,
        modbus_master_transport_transmit,
        NULL);
    if (transaction_result != MBRTUM_TXN_OK) {
        modbus_master_last_protocol_result = transaction_result;
        modbus_master_last_result = MODBUS_MASTER_RESULT_BUILD_ERROR;
        return transaction_result;
    }

    modbus_master_rx_reset_all();
    modbus_master_uart_error_pending = 0u;
    modbus_master_tx_complete_pending = 0u;
    modbus_master_receive_window_open = 0u;
    modbus_master_force_request = 0u;
    modbus_master_cycle_active = 0u;
    modbus_master_test_step = MODBUS_MASTER_TEST_STEP_IDLE;
    modbus_master_last_result = MODBUS_MASTER_RESULT_NONE;
    modbus_master_last_protocol_result = MBRTUM_OK;
    modbus_master_initialized = 1u;
    modbus_master_next_cycle_ms =
        HAL_GetTick() + MODBUS_MASTER_FIRST_REQUEST_DELAY_MS;
    modbus_master_sync_watch_state();

    if (HAL_UART_Receive_IT(&huart3, &modbus_master_uart_rx_byte, 1u) !=
        HAL_OK) {
        ++modbus_uart_error_count;
        modbus_master_initialized = 0u;
        modbus_master_last_result = MODBUS_MASTER_RESULT_UART_ERROR;
        return -1;
    }

    return 0;
}

void modbus_master_request_now(void)
{
    modbus_master_force_request = 1u;
}

void modbus_master_process(void)
{
    uint32_t now;

    if (modbus_master_initialized == 0u) {
        return;
    }

    modbus_master_ensure_receive_armed();
    modbus_master_handle_tx_complete();

    now = HAL_GetTick();

    /* A completed response is processed before timeout polling. */
    modbus_master_process_response_frame(now);
    modbus_master_process_uart_error(now);

    (void)mbrtum_transaction_poll(&modbus_master_transaction, now);
    modbus_master_handle_tx_complete();
    modbus_master_sync_watch_state();
    modbus_master_handle_terminal_transaction(now);
    modbus_master_sync_watch_state();

    if (modbus_master_cycle_active != 0u) {
        return;
    }

    if (modbus_master_force_request != 0u) {
        modbus_master_force_request = 0u;
        modbus_master_start_cycle(now);
        return;
    }

    if (modbus_master_auto_poll_enabled != 0u &&
        modbus_master_time_reached(now, modbus_master_next_cycle_ms)) {
        modbus_master_start_cycle(now);
    }
}

static void modbus_master_print_bits(const char *label,
                                     const uint8_t *values)
{
    uint16_t index;

    printf("%s:", label);
    for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
        printf(" %u", (unsigned int)values[index]);
    }
    printf("\r\n");
}

static void modbus_master_print_registers(const char *label,
                                          const uint16_t *values)
{
    uint16_t index;

    printf("%s:", label);
    for (index = 0u; index < MODBUS_MASTER_TEST_QUANTITY; ++index) {
        printf(" %u", (unsigned int)values[index]);
    }
    printf("\r\n");
}

static void modbus_master_print_last_cycle(void)
{
    printf("Test 0x%02X cycle %lu result=%u txn=%u protocol=%d retries=%lu\r\n",
           (unsigned int)modbus_master_current_test,
           (unsigned long)modbus_master_test_cycles_completed,
           (unsigned int)modbus_master_last_result,
           (unsigned int)modbus_master_transaction_result,
           modbus_master_last_protocol_result,
           (unsigned long)modbus_master_retries);

    if (modbus_master_last_result != MODBUS_MASTER_RESULT_PASS) {
        if (modbus_master_last_result == MODBUS_MASTER_RESULT_EXCEPTION) {
            printf("Modbus exception: 0x%02X\r\n",
                   (unsigned int)modbus_master_last_exception_code);
        } else if (modbus_master_last_result ==
                   MODBUS_MASTER_RESULT_VERIFY_ERROR) {
            printf("Read-back mismatch index=%u expected=%u actual=%u\r\n",
                   (unsigned int)modbus_master_verification_mismatch_index,
                   (unsigned int)modbus_master_verification_expected,
                   (unsigned int)modbus_master_verification_actual);
        }
        return;
    }

    switch (modbus_master_current_test) {
    case MODBUS_MASTER_TEST_FC01:
    case MODBUS_MASTER_TEST_FC05:
    case MODBUS_MASTER_TEST_FC0F:
        modbus_master_print_bits("Coils 0..9", modbus_master_coils);
        break;
    case MODBUS_MASTER_TEST_FC02:
        modbus_master_print_bits("Discrete inputs 0..9",
                                 modbus_master_discrete_inputs);
        break;
    case MODBUS_MASTER_TEST_FC03:
    case MODBUS_MASTER_TEST_FC06:
    case MODBUS_MASTER_TEST_FC10:
        modbus_master_print_registers("Holding registers 0..9",
                                      modbus_master_holding_registers);
        break;
    case MODBUS_MASTER_TEST_FC04:
        modbus_master_print_registers("Input registers 0..9",
                                      modbus_master_input_registers);
        break;
    default:
        break;
    }
}

void app(void)
{
    int init_result;
    uint32_t last_reported_attempts = 0u;
    uint32_t last_reported_cycles = 0u;

    printf("Modbus RTU master all-functions test for STM32F767\r\n");
    printf("USART3: 115200 8N1, full-duplex 3.3 V TTL UART\r\n");
    printf("Slave 1, addresses 0..9, quantity 10\r\n");
    printf("Default selected test: 0x%02X\r\n",
           (unsigned int)modbus_master_selected_test);

    init_result = modbus_master_init();
    if (init_result != 0) {
        printf("Modbus master initialization failed: %d\r\n", init_result);
        Error_Handler();
    }

    printf("Master initialized; first test cycle in 500 ms\r\n");

    while (1) {
        modbus_master_process();

        if (modbus_master_requests_sent != last_reported_attempts) {
            last_reported_attempts = modbus_master_requests_sent;
            printf("Transmission attempt: %lu function=0x%02X step=%u\r\n",
                   (unsigned long)last_reported_attempts,
                   (unsigned int)modbus_master_current_function,
                   (unsigned int)modbus_master_test_step);
        }

        if (modbus_master_test_cycles_completed != last_reported_cycles) {
            last_reported_cycles = modbus_master_test_cycles_completed;
            modbus_master_print_last_cycle();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART3) {
        modbus_master_on_rx_byte_isr(modbus_master_uart_rx_byte);

        if (HAL_UART_Receive_IT(&huart3,
                                &modbus_master_uart_rx_byte,
                                1u) != HAL_OK) {
            ++modbus_uart_error_count;
            modbus_master_uart_error_pending = 1u;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART3) {
        ++modbus_uart_error_count;
        modbus_master_uart_error_pending = 1u;
    }
}

void systick_50us_tick_isr(void)
{
    modbus_master_on_50us_tick_isr();
}
