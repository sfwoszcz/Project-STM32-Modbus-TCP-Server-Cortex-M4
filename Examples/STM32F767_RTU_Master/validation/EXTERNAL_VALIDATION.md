# External STM32F767 Modbus RTU master validation

## Date and scope

External testing was reported on 2026-07-17 using the exact project archive
that had been supplied for validation. The tester stated that no source or
project configuration changes were made.

Validated configuration:

- STM32F767IGTx
- bare metal
- Keil µVision 5.41
- ArmClang 6.22
- USART3, PB10/PB11
- 115200 baud, 8N1
- 3.3 V TTL UART
- RTU master FC03
- slave address 1
- start address 0
- quantity 5

## Controlled endurance result

The clean controlled run produced matching request and valid-response counts:

- 1,002 requests and 1,002 valid responses in one capture
- 1,108 requests and 1,108 valid responses in a later capture

All reported error counters remained zero:

- exception responses
- response timeouts
- transmit errors
- protocol/response errors
- invalid inter-character-gap frames
- receive-buffer overflow
- completed-frame overrun
- unexpected bytes
- UART errors
- UART recoveries

The Keil full rebuild reported zero errors and zero warnings.

## Frame evidence

The communication trace repeatedly showed:

```text
Request:  01 03 00 00 00 05 85 C9
Response: 01 03 0A 00 00 00 00 00 00 00 00 00 00 24 B6
```

The response is 15 bytes, matching FC03 with five 16-bit registers.

## Tested archive identity

Returned archive filename:

```text
modbus-f767-rtu-master-init(1).zip
```

SHA-256:

```text
eb448f5c86b88fed112e4c6148263b498a4cd7ee0acdc2ea77b01df6d8a6f852
```

The returned archive hash matched the supplied validation archive, supporting
the statement that the tester built and exercised the unchanged package.

## Interpretation

This result validates the listed FC03 TTL-UART hardware-smoke-test scope. It is
not evidence of physical RS-485 behavior, DE/RE turnaround, multidrop operation,
electrical-noise immunity, all supported Modbus function codes, or the future
portable retry/transaction engine.

## Clean repository adaptation

The exact externally tested archive compiled its own local
`app/src/platform_stm32.c`. That file supplied an optional RTOS lock
implementation and a weak `sys_now()` fallback. The tested RTU master sources
did not reference `sys_now()`, and the build did not define `WITH_RTOS`.

The repository's canonical `App/src/platform_stm32.c` is instead the TCP/lwIP
platform adapter and includes lwIP types. Referencing it from the bare-metal RTU
Keil target would change the tested source and could require unrelated lwIP
headers.

For the clean GitHub example, the `platform_stm32.c` entry is therefore removed
from the Keil target. This is a repository cleanup, not a claim that the
cleaned project itself was independently rebuilt by the external tester. The
external evidence remains tied to the archive SHA-256 recorded above.

## Partial eight-function hardware revalidation

The selectable eight-function candidate was externally rebuilt and exercised
on the STM32F767IGTx on 2026-07-19 and 2026-07-20.

Updated candidate configuration:

- STM32F767IGTx
- bare metal
- Keil µVision 5.41.0.0
- ArmClang 6.22
- USART3 on PB10 TX and PB11 RX
- 115200 baud, 8N1
- 3.3 V TTL UART
- slave address 1
- start address 0
- quantity 10 for the read tests

The full Keil rebuild completed with zero errors and zero warnings.

### FC01 Read Coils

FC01 was tested over addresses 0 through 9 with quantity 10.

The tester exercised both all-zero and all-one coil patterns. The values
reported by the STM32 master matched the slave display and communication trace.

The reported endurance run exceeded 1,000 cycles and 10 minutes. The captured
output showed successful protocol results and zero retries.

### FC02 Read Discrete Inputs

FC02 was tested over addresses 0 through 9 with quantity 10.

One recorded pattern was:

```text
1, 1, 0, 0, 0, 0, 0, 0, 0, 0
```

The STM32 `modbus_master_discrete_inputs` array matched the slave display.

The reported endurance run reached at least 1,000 cycles and exceeded
10 minutes. The captured output showed successful protocol results and zero
retries.

### FC03 Read Holding Registers

FC03 was tested over addresses 0 through 9 with quantity 10.

The reported register values were:

```text
1, 2, 3, 4, 5, 6, 7, 8, 65534, 65535
```

The final two values correspond to signed test values -2 and -1 represented as
the Modbus 16-bit values `0xFFFE` and `0xFFFF`.

The reported endurance run exceeded 1,000 cycles and 10 minutes. The
communication trace, STM32 Watch values, and terminal output were consistent,
with successful protocol results and zero retries.

### FC04 Read Input Registers

FC04 was tested over addresses 0 through 9 with quantity 10.

The recorded slave and STM32 values were all zero. Repeated FC04 request and
25-byte response frames were visible in the communication trace.

The reported run reached at least 571 cycles and exceeded 6 minutes. The
captured counters showed zero invalid-gap frames, receive-buffer overflows,
completed-frame overruns, unexpected bytes, UART errors, and UART recoveries.

### Current interpretation

The updated STM32F767 candidate is externally hardware validated for:

- FC01 Read Coils
- FC02 Read Discrete Inputs
- FC03 Read Holding Registers
- FC04 Read Input Registers

The following write paths remain host verified but do not yet have external
hardware validation:

- FC05 Write Single Coil
- FC06 Write Single Register
- FC0F Write Multiple Coils
- FC10 Write Multiple Registers

The tester's available slave software could exercise only the four read data
tables and could not execute the four requested write tests. The pending write
tests are therefore an external test-environment limitation, not reported
function failures.

The portable write request builders, response validation, transaction handling,
and automatic read-back paths remain covered by strict host tests, sanitizers,
and the eight-function adapter simulation.

This evidence still covers 3.3 V TTL UART only. It does not validate a physical
RS-485 transceiver, DE/RE direction control, multidrop operation, or electrical
noise performance.
