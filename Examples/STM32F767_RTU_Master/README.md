# STM32F767 Modbus RTU master example

This directory contains the STM32F767IGTx bare-metal Keil project for the
repository's portable Modbus RTU master implementation.

The original FC03-only configuration was externally compiled and exercised on
physical hardware. The current eight-function candidate keeps that proven
USART3 receive/timing adapter and integrates the portable RTU master transaction
engine.

The Keil project references the canonical repository sources:

- `../../../App/include`
- `../../../App/src`

Do not copy a second Modbus implementation into this directory.

## Board and serial configuration

- MCU: STM32F767IGTx
- Toolchain used for the historical validation: Keil µVision 5.41, ArmClang 6.22
- Operating system: none (bare metal)
- UART: USART3
- Pins: PB10 TX, PB11 RX
- Serial format: 115200 baud, 8 data bits, no parity, 1 stop bit
- Electrical interface: 3.3 V TTL UART with crossed TX/RX and common GND
- Modbus role: RTU master
- Slave address: 1
- Test range: addresses 0 through 9

The project name `CAN` is inherited from the supplied CubeMX/Keil board project.
It is retained to avoid unnecessary changes to the tested project structure.

## Current eight-function candidate

Exactly one test is selected at a time:

| Selector | Function |
|---|---|
| `MODBUS_MASTER_TEST_FC01` | Read Coils |
| `MODBUS_MASTER_TEST_FC02` | Read Discrete Inputs |
| `MODBUS_MASTER_TEST_FC03` | Read Holding Registers |
| `MODBUS_MASTER_TEST_FC04` | Read Input Registers |
| `MODBUS_MASTER_TEST_FC05` | Write Single Coil |
| `MODBUS_MASTER_TEST_FC06` | Write Single Register |
| `MODBUS_MASTER_TEST_FC0F` | Write Multiple Coils |
| `MODBUS_MASTER_TEST_FC10` | Write Multiple Registers |

Read and multiple-write tests use starting address 0 and quantity 10.

Write tests include automatic read-back verification:

- FC05 followed by FC01
- FC06 followed by FC03
- FC0F followed by FC01
- FC10 followed by FC03

A write-test cycle therefore contains two successful transactions: the write
acknowledgement and the verification read.

## Selecting a test

The compile-time default is in `app/app.h`:

```c
#define MODBUS_MASTER_ACTIVE_TEST MODBUS_MASTER_TEST_FC03
```

The selection may also be changed through
`modbus_master_selected_test` in Keil Watch while
`modbus_master_test_step == MODBUS_MASTER_TEST_STEP_IDLE`.

Use only one active test at a time. There is no need to comment out seven
function calls.

## Transaction policy

The example uses the portable `mbrtum_transaction_*()` engine with:

- one outstanding request
- 1,000 ms response timeout
- two retries
- 100 ms retry delay
- retry of transmit-start failures
- strict complete-frame response validation
- bounded diagnostics and no dynamic allocation

UART byte reception and T1.5/T3.5 frame assembly remain board-specific.

## Build

Open:

```text
MDK-ARM/CAN.uvprojx
```

Perform a full **Rebuild** before flashing.

The Keil target references:

- `modbus_rtu_master.c`
- `modbus_rtu_master_transaction.c`
- their canonical headers in `App/include`

Keep this example inside the repository at its current relative path.

## Validation status

The historical FC03 configuration has external Keil and hardware evidence in
`validation/EXTERNAL_VALIDATION.md`.

The current eight-function candidate has passed the repository regression suite,
strict host-side adapter compilation/simulation, and Keil project XML parsing.
It was also rebuilt externally with ArmClang 6.22 with zero errors and zero
warnings. FC01, FC02, FC03, and FC04 are externally hardware validated.
FC05, FC06, FC0F, and FC10 remain host verified with external hardware
validation pending because the available slave software could not exercise
the write requests.

## Safety and scope

FC05, FC06, FC0F, and FC10 modify slave data in addresses 0 through 9. Test only
against a slave whose coils and holding registers in that range are safe to
change.

This project uses 3.3 V TTL UART. It does not implement an RS-485 transceiver or
DE/RE direction control.

See `BUILD_AND_TEST.md` for the test procedure.
