# Build and hardware test

## 1. Keil build

1. Keep `Examples/STM32F767_RTU_Master` inside the repository.
2. Open `MDK-ARM/CAN.uvprojx` in Keil MDK-ARM.
3. Confirm that the selected device is STM32F767IGTx.
4. Confirm that the `Modbus` group includes
   `modbus_rtu_master_transaction.c`.
5. Perform a full **Rebuild**.
6. Record the error and warning counts.
7. Flash the resulting image to the target board.

The historical FC03 validation used µVision 5.41 and ArmClang 6.22 and reported
zero errors and zero warnings. The updated eight-function candidate was also
rebuilt externally with ArmClang 6.22 with zero errors and zero warnings.
FC01 through FC04 now have external STM32F767 hardware evidence. FC05, FC06,
FC0F, and FC10 remain pending because the tester's slave software could not
exercise the write requests.

## 2. Serial connection

USART3 is configured as 115200 baud, 8N1:

- PB10 (TX) to slave RX
- PB11 (RX) to slave TX
- GND to GND

Use compatible 3.3 V TTL UART levels. Do not connect true RS-232 voltage levels
directly to STM32 GPIO pins. This example does not provide an RS-485
transceiver or DE/RE control.

## 3. Slave test map

The agreed test map is:

- coils: addresses 0 through 9
- discrete inputs: addresses 0 through 9
- holding registers: addresses 0 through 9
- input registers: addresses 0 through 9
- slave address: 1

FC05, FC06, FC0F, and FC10 modify coils or holding registers in this range.
Use a test slave on which these writes are safe.

## 4. Select one test

The default selector is in `app/app.h`:

```c
#define MODBUS_MASTER_ACTIVE_TEST MODBUS_MASTER_TEST_FC03
```

Available values:

```text
MODBUS_MASTER_TEST_FC01
MODBUS_MASTER_TEST_FC02
MODBUS_MASTER_TEST_FC03
MODBUS_MASTER_TEST_FC04
MODBUS_MASTER_TEST_FC05
MODBUS_MASTER_TEST_FC06
MODBUS_MASTER_TEST_FC0F
MODBUS_MASTER_TEST_FC10
```

Alternatively, change `modbus_master_selected_test` in Keil Watch only while:

```text
modbus_master_test_step == MODBUS_MASTER_TEST_STEP_IDLE
```

Test one selector at a time. There is no need to comment out source calls.

## 5. Test behavior

The firmware waits 500 ms after initialization and then starts one test cycle
per second while `modbus_master_auto_poll_enabled` is nonzero.

Read tests:

- FC01 reads 10 coils from address 0.
- FC02 reads 10 discrete inputs from address 0.
- FC03 reads 10 holding registers from address 0.
- FC04 reads 10 input registers from address 0.

Write tests:

- FC05 writes coil 0 and then reads coils 0 through 9 with FC01.
- FC06 writes holding register 0 and then reads registers 0 through 9 with FC03.
- FC0F writes 10 coils from address 0 and then reads them with FC01.
- FC10 writes 10 holding registers from address 0 and then reads them with FC03.

The default write values are visible and editable in Keil Watch while no test
cycle is active.

## 6. Primary Watch variables

Configuration and state:

```text
modbus_master_selected_test
modbus_master_current_test
modbus_master_test_step
modbus_master_last_result
modbus_master_transaction_state
modbus_master_transaction_result
modbus_master_last_protocol_result
modbus_master_last_exception_code
modbus_master_current_function
```

Cycle and transaction counters:

```text
modbus_master_test_cycles_started
modbus_master_test_cycles_completed
modbus_master_test_cycles_passed
modbus_master_test_cycles_failed
modbus_master_transactions_started
modbus_master_requests_sent
modbus_master_valid_responses
modbus_master_timeouts
modbus_master_retries
modbus_master_transmit_errors
modbus_master_response_errors
```

Framing and UART counters:

```text
modbus_master_invalid_gap_frames
modbus_master_overflow_frames
modbus_master_overrun_frames
modbus_master_unexpected_bytes
modbus_uart_error_count
modbus_uart_recovery_count
```

Read-back verification:

```text
modbus_master_verification_passes
modbus_master_verification_failures
modbus_master_verification_mismatch_index
modbus_master_verification_expected
modbus_master_verification_actual
```

Data arrays:

```text
modbus_master_coils[10]
modbus_master_discrete_inputs[10]
modbus_master_holding_registers[10]
modbus_master_input_registers[10]
```

## 7. Interpreting counters

`modbus_master_requests_sent` counts transmission attempts, including retries.

`modbus_master_valid_responses` counts successful transactions. A successful
read cycle normally contributes one transaction. A successful write cycle
contributes two transactions because it includes read-back verification.

Therefore, do not require the request-attempt count to equal the completed cycle
count.

A clean run for the selected test should show:

```text
modbus_master_test_cycles_failed == 0
modbus_master_timeouts == 0
modbus_master_retries == 0
modbus_master_transmit_errors == 0
modbus_master_response_errors == 0
modbus_master_verification_failures == 0
modbus_master_invalid_gap_frames == 0
modbus_master_overflow_frames == 0
modbus_master_overrun_frames == 0
modbus_uart_error_count == 0
```

For FC01 through FC04, verify that the displayed values match the slave.

For FC05, FC06, FC0F, and FC10, require both a valid write acknowledgement and
a passing automatic read-back.

## 8. Recommended validation order

Run and record each selector separately:

1. FC01
2. FC02
3. FC03
4. FC04
5. FC05
6. FC06
7. FC0F
8. FC10

For every test, record:

- Keil build result
- selected function
- completed and passed cycles
- valid responses
- exceptions
- timeouts and retries
- transport, response, framing, and UART errors
- write read-back result where applicable
- a communication trace or screenshot

## 9. Architecture boundary

`app/app.c` is the STM32F767 adapter and hardware-validation harness.

Portable request building, response validation, deadlines, retries, and
transaction state remain in the canonical repository sources. Physical RS-485
support remains a separate future transport stage.
