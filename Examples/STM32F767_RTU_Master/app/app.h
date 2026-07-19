#ifndef APP_H_
#define APP_H_

#include "main.h"
#include "usart.h"

#include "modbus_rtu.h"
#include "modbus_rtu_master.h"
#include "modbus_rtu_master_transaction.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_MASTER_TEST_START_ADDRESS 0u
#define MODBUS_MASTER_TEST_QUANTITY 10u

/*
 * Select the default hardware test here. The same selection can also be
 * changed at run time through modbus_master_selected_test in Keil Watch while
 * no test cycle is active.
 */
#ifndef MODBUS_MASTER_ACTIVE_TEST
#define MODBUS_MASTER_ACTIVE_TEST MODBUS_MASTER_TEST_FC03
#endif

typedef enum {
    MODBUS_MASTER_TEST_FC01 = MBRTUM_FC_READ_COILS,
    MODBUS_MASTER_TEST_FC02 = MBRTUM_FC_READ_DISCRETE_INPUTS,
    MODBUS_MASTER_TEST_FC03 = MBRTUM_FC_READ_HOLDING_REGISTERS,
    MODBUS_MASTER_TEST_FC04 = MBRTUM_FC_READ_INPUT_REGISTERS,
    MODBUS_MASTER_TEST_FC05 = MBRTUM_FC_WRITE_SINGLE_COIL,
    MODBUS_MASTER_TEST_FC06 = MBRTUM_FC_WRITE_SINGLE_REGISTER,
    MODBUS_MASTER_TEST_FC0F = MBRTUM_FC_WRITE_MULTIPLE_COILS,
    MODBUS_MASTER_TEST_FC10 = MBRTUM_FC_WRITE_MULTIPLE_REGISTERS
} modbus_master_test_t;

typedef enum {
    MODBUS_MASTER_TEST_STEP_IDLE = 0,
    MODBUS_MASTER_TEST_STEP_REQUEST = 1,
    MODBUS_MASTER_TEST_STEP_VERIFY_READBACK = 2
} modbus_master_test_step_t;

typedef enum {
    MODBUS_MASTER_RESULT_NONE = 0,
    MODBUS_MASTER_RESULT_PASS = 1,
    MODBUS_MASTER_RESULT_BUILD_ERROR = 2,
    MODBUS_MASTER_RESULT_START_ERROR = 3,
    MODBUS_MASTER_RESULT_EXCEPTION = 4,
    MODBUS_MASTER_RESULT_TIMEOUT = 5,
    MODBUS_MASTER_RESULT_TRANSPORT_ERROR = 6,
    MODBUS_MASTER_RESULT_RESPONSE_ERROR = 7,
    MODBUS_MASTER_RESULT_DECODE_ERROR = 8,
    MODBUS_MASTER_RESULT_VERIFY_ERROR = 9,
    MODBUS_MASTER_RESULT_CANCELLED = 10,
    MODBUS_MASTER_RESULT_UART_ERROR = 11,
    MODBUS_MASTER_RESULT_INVALID_TEST = 12
} modbus_master_result_t;

/* Keil Watch-friendly configuration and current state. */
extern volatile uint32_t systick_count;
extern volatile modbus_master_test_t modbus_master_selected_test;
extern volatile modbus_master_test_t modbus_master_current_test;
extern volatile modbus_master_test_step_t modbus_master_test_step;
extern volatile modbus_master_result_t modbus_master_last_result;
extern volatile mbrtum_transaction_state_t modbus_master_transaction_state;
extern volatile mbrtum_transaction_result_t modbus_master_transaction_result;
extern volatile int modbus_master_last_protocol_result;
extern volatile uint8_t modbus_master_last_exception_code;
extern volatile uint16_t modbus_master_last_response_length;
extern volatile uint8_t modbus_master_current_function;
extern volatile uint8_t modbus_master_auto_poll_enabled;

/* Logical test-cycle and transaction diagnostics. */
extern volatile uint32_t modbus_master_test_cycles_started;
extern volatile uint32_t modbus_master_test_cycles_completed;
extern volatile uint32_t modbus_master_test_cycles_passed;
extern volatile uint32_t modbus_master_test_cycles_failed;
extern volatile uint32_t modbus_master_transactions_started;
extern volatile uint32_t modbus_master_requests_sent;
extern volatile uint32_t modbus_master_valid_responses;
extern volatile uint32_t modbus_master_exception_responses;
extern volatile uint32_t modbus_master_timeouts;
extern volatile uint32_t modbus_master_retries;
extern volatile uint32_t modbus_master_transmit_errors;
extern volatile uint32_t modbus_master_response_errors;
extern volatile uint32_t modbus_master_unrelated_responses;
extern volatile uint32_t modbus_master_late_responses;
extern volatile uint32_t modbus_master_cancelled_transactions;
extern volatile uint32_t modbus_master_build_errors;
extern volatile uint32_t modbus_master_decode_errors;
extern volatile uint32_t modbus_master_verification_passes;
extern volatile uint32_t modbus_master_verification_failures;

/* UART receive/framing diagnostics retained from the validated FC03 project. */
extern volatile uint32_t modbus_master_received_bytes;
extern volatile uint32_t modbus_master_detected_frames;
extern volatile uint32_t modbus_master_invalid_gap_frames;
extern volatile uint32_t modbus_master_overflow_frames;
extern volatile uint32_t modbus_master_overrun_frames;
extern volatile uint32_t modbus_master_unexpected_bytes;
extern volatile uint32_t modbus_uart_error_count;
extern volatile uint32_t modbus_uart_recovery_count;

/* Read results for addresses 0..9. */
extern uint8_t modbus_master_coils[MODBUS_MASTER_TEST_QUANTITY];
extern uint8_t modbus_master_discrete_inputs[MODBUS_MASTER_TEST_QUANTITY];
extern uint16_t modbus_master_holding_registers[MODBUS_MASTER_TEST_QUANTITY];
extern uint16_t modbus_master_input_registers[MODBUS_MASTER_TEST_QUANTITY];

/* Write values may be edited in Keil Watch while no test cycle is active. */
extern volatile uint8_t modbus_master_single_coil_write_value;
extern volatile uint16_t modbus_master_single_register_write_value;
extern volatile uint8_t modbus_master_multiple_coil_write_values[MODBUS_MASTER_TEST_QUANTITY];
extern volatile uint16_t modbus_master_multiple_register_write_values[MODBUS_MASTER_TEST_QUANTITY];

/* First read-back mismatch, or index 0xFFFF when the last check matched. */
extern volatile uint16_t modbus_master_verification_mismatch_index;
extern volatile uint16_t modbus_master_verification_expected;
extern volatile uint16_t modbus_master_verification_actual;

/* Full transaction context is visible for advanced Keil Watch inspection. */
extern mbrtum_transaction_t modbus_master_transaction;

void app(void);
int modbus_master_init(void);
void modbus_master_process(void);
void modbus_master_request_now(void);
void systick_50us_tick_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
