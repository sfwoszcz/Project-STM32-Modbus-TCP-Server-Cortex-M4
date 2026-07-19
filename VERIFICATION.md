# Verification report

Verification performed on 2026-07-19.

## Successful checks

| Check | Result |
|---|---|
| GCC strict C11 build | Pass |
| Clang strict C11 build | Pass |
| AddressSanitizer + UndefinedBehaviorSanitizer | Pass |
| Register-map self-test | 5/5 pass |
| Modbus CRC-16 known-vector tests | Pass |
| Shared PDU unit tests | Pass |
| Modbus RTU ADU tests | Pass |
| Modbus RTU master request/response tests | Pass |
| Modbus RTU master transaction-state tests | Pass |
| Modbus RTU 50 us timing/state tests | Pass |
| Modbus TCP ADU regression tests | Pass |
| POSIX TCP integration smoke test | Pass |
| lwIP transport compile check | Pass |
| CMake configure/build/CTest (8 tests) | Pass |
| SVG XML validation and PNG rendering | Pass |

The strict warning set is:

```text
-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror
```

## RTU slave coverage

The host RTU suite verifies:

- CRC calculation, wire byte order, and corruption rejection
- all eight supported function codes through RTU framing
- normal and exception response CRCs
- configurable slave addressing
- silent discard of wrong-address and invalid-CRC frames
- broadcast writes with no response
- broadcast read and unsupported-function suppression
- truncated, oversized, minimum-size, and maximum-size frames
- output-capacity failure without applying a write
- 9600, 19200, and 115200 timing thresholds
- exact T1.5 acceptance and T1.5-to-T3.5 rejection
- T3.5 frame publication and two-buffer handoff
- pending-frame overrun reporting
- receive-buffer overflow and recovery
- main-loop transmit dispatch and transmit-error reporting
- API argument and configured-address validation

## RTU master coverage

The host RTU master suite verifies:

- FC01, FC02, FC03, FC04, FC05, FC06, FC0F, and FC10 request generation
- low-byte-first request CRC generation
- unicast requests and broadcast-write behavior
- quantity, value, capacity, and 16-bit address-range validation
- maximum-size FC0F and FC10 requests
- complete response length, CRC, address, and function validation
- Modbus exception response decoding
- exact read byte-count validation
- zero-valued unused packed-bit padding
- exact single-write address/value acknowledgements
- exact multiple-write address/quantity acknowledgements
- zero-copy bit and register decoding with index validation
- malformed response and API argument rejection

## RTU master transaction coverage

The host transaction suite verifies:

- initialization, configuration, and uninitialized-context guards
- consistency between the request descriptor and complete request ADU
- request address, function, length, and CRC validation before transmission
- transaction-owned immutable copies of the request descriptor and ADU
- asynchronous transmit acceptance, completion, and failure events
- one-outstanding-request and busy-state enforcement
- wrap-safe response deadlines and late-response rejection
- configurable retry counts and retry delays
- poll-driven zero-delay retries without recursive retransmission
- retry success and retry-exhaustion results
- optional retry of transport failures
- broadcast completion after successful transmission without response waiting
- valid Modbus exception completion without automatic retry
- malformed-response, unrelated-response, and response-error accounting
- cancellation and tick-counter wraparound behavior

## Scope

The shared Modbus PDU core, backward-compatible TCP ADU wrapper, portable CRC-16 implementation, complete-frame RTU slave ADU wrapper, portable complete-frame RTU master request/response core, portable RTU master transaction engine, host demonstration, tests, and lwIP-facing application sources are verified.

The portable RTU byte-receive/timing state machine, fixed 50 microsecond tick API, T1.5/T3.5 frame detection, buffering, and recovery are host verified. The STM32 UART/timer adapter and physical hardware validation remain outside this stage.

A final STM32 firmware ELF/BIN/HEX cannot be produced without board-specific STM32CubeMX output for the selected MCU and board, including startup code, linker script, HAL/CMSIS, Ethernet MAC/PHY configuration, UART configuration, and generated lwIP port files.

Integration and current RTU scope are documented in `README.md`, `docs/modbus-rtu-core.md`, `docs/modbus-rtu-master-core.md`, `docs/modbus-rtu-master-transaction.md`, `docs/modbus-rtu-timing.md`, and `Examples/stm32_cube_main.c`.

The transaction engine is host verified and is now integrated into the STM32F767 TTL-UART example as a selectable eight-function candidate. External ArmClang/Keil compilation and physical hardware revalidation of that updated example are still pending.

## External STM32F767 hardware validation

The portable Modbus RTU slave was integrated into an STM32F767IGTx
CubeMX/Keil project and tested on physical hardware using USART3 at
115200 baud, 8N1.

All supported RTU slave function codes were demonstrated successfully:
FC01, FC02, FC03, FC04, FC05, FC06, FC0F, and FC10.

Long-duration FC03 and FC10 stress testing completed without RTU processing,
buffering, or transmission errors after an unstable PC USB serial connection
was identified and corrected.

See [`docs/stm32f767-rtu-validation.md`](docs/stm32f767-rtu-validation.md)
for the detailed external validation record.

## External STM32F767 RTU master validation

On 2026-07-17, the portable Modbus RTU master core was externally tested on an
STM32F767IGTx using a bare-metal STM32Cube/Keil project.

Validated configuration:

- Keil µVision 5.41.0.0
- ArmClang 6.22
- operating system: none
- USART3
- PB10 TX and PB11 RX
- 115200 baud, 8 data bits, no parity, 1 stop bit
- Modbus RTU slave address 1
- FC03 Read Holding Registers
- starting address 0
- quantity 5

The controlled run recorded 1,002 requests and 1,002 valid responses with:

- zero timeouts
- zero transmit errors
- zero response/protocol errors
- zero exception responses
- zero invalid RTU gap frames
- zero receive-buffer overflow frames
- zero completed-frame overruns
- zero unexpected bytes
- zero UART errors

A later capture showed 1,108 requests and 1,108 valid responses with the same
zero-error counters. The Keil rebuild completed with zero errors and zero
warnings.

The external tester confirmed that the supplied project was compiled and tested
without source-code or project-setting modifications.

This validation covers the portable FC03 request/response core and the
STM32F767 USART3 bare-metal adapter over 3.3 V TTL UART. It does not validate a
physical RS-485 transceiver, DE/RE direction control, multidrop operation, or
electrical-noise performance.

The cleaned repository example is located at
`Examples/STM32F767_RTU_Master/`. Its Keil target references the canonical
portable sources in `App/`. The unrelated TCP/lwIP
`App/src/platform_stm32.c` is intentionally excluded from the bare-metal RTU
target. The validation directory contains the test record, selected evidence,
the tested-source checksum manifest, and a line-ending-independent verification
script.
