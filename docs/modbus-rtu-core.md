# Portable Modbus RTU ADU core

This stage adds complete-frame Modbus RTU processing without coupling the protocol core to an MCU, UART driver, timer, DMA engine, RTOS, or physical transceiver.

## Scope

Implemented now:

- Modbus CRC-16 calculation
- low-byte-first CRC encoding and validation
- RTU ADU length validation
- configurable unicast slave address from 1 through 247
- address 0 broadcast handling
- silent discard of invalid CRC frames and frames for another slave
- shared processing of function codes `01`, `02`, `03`, `04`, `05`, `06`, `0F`, `10`, `14`, `15`, `16`, `17`, `18`, and `2B/0E`
- RTU-only FC11 Report Server ID processing with copied fixed-capacity data
- normal and exception response framing
- host tests for all supported function codes and RTU-specific behavior

Implemented by the following portable timing stage:

- single-byte receive events
- fixed 50 microsecond timing events
- T1.5/T3.5 frame-boundary detection
- two-buffer receive ownership and explicit overrun reporting
- overflow and invalid-gap recovery
- main-loop transmission callbacks

Still not implemented:

- STM32 HAL/CubeMX UART and timer glue
- physical serial hardware validation
- RS-485 DE/RE control

## Frame format

A complete RTU ADU is represented as:

```text
+---------+----------------------+---------+----------+
| Address | Modbus request PDU   | CRC low | CRC high |
+---------+----------------------+---------+----------+
   1 byte      1 to 253 bytes       1 byte    1 byte
```

The maximum RTU ADU size is 256 bytes.

## API

```c
int mbrtu_process_adu(uint8_t slave_address,
                      const uint8_t *request_adu,
                      size_t request_adu_len,
                      uint8_t *response_adu,
                      size_t response_capacity,
                      size_t *response_adu_len);
```

Return values:

- `MBRTU_RESPONSE_READY`: a response frame is ready to transmit
- `MBRTU_NO_RESPONSE`: the frame was ignored or was a broadcast
- negative value: invalid API use, invalid configured slave address, or insufficient response capacity

`response_adu_len` is set to zero before frame validation, so callers can use it safely after any non-response result.

## Processing rules

For a unicast request addressed to the configured slave:

1. Validate the RTU frame length.
2. Validate the slave address.
3. Validate the received CRC.
4. Pass the request PDU to `mb_process_pdu()`.
5. Prefix the configured address.
6. Append a newly calculated CRC, low byte first.

For a broadcast request at address 0:

- supported write-only functions are processed
- no response is generated
- read functions, FC11, FC20, FC23, FC24, FC43/14, and unsupported functions are ignored
- FC21 and FC22 are processed as broadcast writes and produce no response
- malformed writes do not modify the register map

Frames with addresses 248 through 255 are reserved and are not accepted as configured slave addresses.

## FC22 Mask Write Register

FC22 is processed by the shared PDU engine for Modbus TCP and RTU. It performs
one locked holding-register read-modify-write using the standard AND/OR mask
formula and returns an exact request echo for unicast requests. The RTU wrapper
also accepts address-zero FC22 broadcasts, applies the validated update, and
suppresses the response. See
[`modbus-fc22-mask-write-register.md`](modbus-fc22-mask-write-register.md).

## FC23 shared processing

FC23 Read/Write Multiple Registers uses the same shared PDU implementation for
Modbus TCP and RTU unicast requests. The core validates both holding-register
ranges, quantities, byte count, exact request length, and response capacity
before changing the map. The write is then performed before the read. See
[`modbus-fc23.md`](modbus-fc23.md) for the complete contract.

## FC24 Read FIFO Queue

FC24 is processed by the shared PDU engine for Modbus TCP and RTU unicast
requests. It reads a non-destructive snapshot from a configured
application-owned register block, returns a two-byte Byte Count and FIFO Count,
and includes up to 31 queued register values. Unknown pointer addresses return
Illegal Data Address, counts above 31 return Illegal Data Value, and
inconsistent storage returns Server Device Failure. See
[`modbus-fc24-read-fifo-queue.md`](modbus-fc24-read-fifo-queue.md).

## CRC-16

`mb_crc16()` uses the Modbus serial-line algorithm:

- initial value `0xFFFF`
- reflected polynomial `0xA001`
- eight right shifts per input byte
- low-order CRC byte transmitted first

The implementation is table-free to keep ROM use and provenance simple. A future optimized table implementation can be added behind the same API if measured performance requires it.

## Buffer ownership

The caller owns the request and response buffers. The RTU core performs no dynamic allocation and retains no pointer after the function returns.

The request and response buffers should not overlap. A 256-byte response buffer supports every legal Modbus RTU response.

## Byte and timing layer

The portable byte/timing layer is now implemented through:

- `mbrtu_init()`
- `mbrtu_on_rx_byte_isr()`
- `mbrtu_on_50us_tick_isr()`
- `mbrtu_poll()`

See [`modbus-rtu-timing.md`](modbus-rtu-timing.md) for timing calculations, buffer ownership, ISR/main-loop responsibilities, diagnostics, and the remaining STM32 hardware-integration boundary.

## Authoritative references

- Modbus Application Protocol Specification V1.1b3: <https://www.modbus.org/file/secure/modbusprotocolspecification.pdf>
- Modbus Serial Line Protocol and Implementation Guide V1.02: <https://www.modbus.org/file/secure/modbusoverserial.pdf>

## FC11 Report Server ID

FC11 is dispatched exclusively by the RTU wrapper and is deliberately absent
from the shared PDU/TCP path. Applications link
`App/src/modbus_rtu_server_id.c`, define `MBRTU_ENABLE_SERVER_ID=1`, and
configure a copied, device-specific Server ID of 1–250 bytes, run status, and
optional Additional Data with `mbrtu_server_id_configure()`. Address-zero requests are ignored. See
[`modbus-fc11-report-server-id.md`](modbus-fc11-report-server-id.md).

## FC20 Read File Record

FC20 is processed by the shared PDU engine and is therefore available through
both Modbus TCP and Modbus RTU. Applications configure up to
`MB_FILE_RECORD_MAX_FILES` descriptors with file number, record pointer, and
record count. The descriptors are copied into fixed-capacity state, while the
record arrays remain application-owned.

Each request contains one or more seven-byte subrequests with reference type
`0x06`, file number, record number, and register count. The core validates the
complete request and combined response size before copying any record data.
See [`modbus-fc20-read-file-record.md`](modbus-fc20-read-file-record.md).

## FC21 Write File Record

FC21 uses the same configured application-owned file map as FC20. The shared
PDU engine validates every variable-length subrequest, all file and record
ranges, the complete request-data extent, and response capacity before changing
any file data. Successful requests are applied in request order and return an
exact request echo.

The RTU wrapper accepts FC21 at broadcast address zero because it is write-only.
Broadcast data is validated and applied, while the response is suppressed. See
[`modbus-fc21-write-file-record.md`](modbus-fc21-write-file-record.md).

## FC43/14 Read Device Identification

FC43/14 is processed by the shared PDU engine and is therefore available to
both Modbus TCP and unicast Modbus RTU requests. The application configures a
fixed-capacity object model through `mb_device_id_configure()`. RTU address-zero
broadcast requests are silently ignored because the function is read-only.
Object-list pagination, conformity levels, maximum object sizing, and exception
behavior are documented in `docs/modbus-fc43-device-identification.md`.
