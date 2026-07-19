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

## Pending eight-function revalidation

On 2026-07-19, the example was extended from the historical FC03-only harness to
a selectable eight-function master candidate:

- FC01, FC02, FC03, and FC04 reads
- FC05, FC06, FC0F, and FC10 writes
- automatic read-back verification for every write test
- portable transaction-engine timeout, retry, and diagnostic handling
- addresses 0 through 9 with quantity 10 where applicable

This updated candidate has not yet replaced the historical evidence above.
ArmClang/Keil compilation and physical STM32F767 testing are still pending.
