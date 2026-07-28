# FC11 Report Server ID

FC11 (`0x11`) is a serial-line-only Modbus service that reports a
vendor-specific Server ID, the current run status, and optional additional
data. It is intentionally unavailable through Modbus TCP.

## Wire format

The request PDU contains only the function code:

```text
+----------+
| 0x11     |
+----------+
```

A normal response PDU is:

```text
+----------+------------+--------------------+------------+-----------------+
| 0x11     | Byte Count | Server ID (N bytes)| Run Status | Additional Data |
+----------+------------+--------------------+------------+-----------------+
```

The Server ID length is device-specific. `Byte Count` includes all Server ID
bytes, the one-byte Run Indicator Status, and all Additional Data bytes. Run
Indicator Status is `0x00` for OFF or `0xFF` for ON.

The combined response data may contain at most 251 bytes. Therefore, the
Server ID may contain from 1 through 250 bytes, and the combined Server ID and
Additional Data may contain at most 250 bytes. A maximum response produces a
253-byte Modbus PDU and a 256-byte RTU ADU.

## Slave configuration

Include:

```c
#include "modbus_rtu_server_id.h"
```

Configure the response before processing FC11 requests:

```c
static const uint8_t server_id[] = {0x42u, 0x01u};
static const uint8_t details[] = "STM32F4 Modbus RTU";

int result = mbrtu_server_id_configure(
    server_id,
    sizeof(server_id),
    MBRTU_SERVER_ID_RUN_STATUS_ON,
    details,
    sizeof(details) - 1u);
```

The implementation copies the Server ID and Additional Data into one
fixed-capacity internal response buffer. The caller's input buffers do not
need to remain valid after configuration. Failed configuration leaves the
previous response active.

Use `mbrtu_server_id_clear()` to remove the active response and
`mbrtu_server_id_is_configured()` to query its state. An addressed FC11
request received while no response is configured returns Server Device
Failure (`0x04`).

Repository builds define `MBRTU_ENABLE_SERVER_ID=1` and link
`App/src/modbus_rtu_server_id.c`. Existing integrations that compile the
pre-FC11 RTU source list remain link-compatible and continue returning Illegal
Function for FC11 until the source and definition are added.

## RTU behavior

- unicast request with exactly one PDU byte: normal response;
- malformed request data: Illegal Data Value (`0x03`);
- no configured response: Server Device Failure (`0x04`);
- address-zero broadcast: silently ignored;
- wrong address, invalid CRC, or invalid RTU length: silently ignored;
- listen-only diagnostics mode: response suppressed by the existing
  diagnostics state machine.

FC11 is dispatched directly by the RTU layer. It is not added to
`mb_process_pdu()`, so `mbtcp_process_adu()` continues to return Illegal
Function for function `0x11`.

## RTU master API

The Server ID length is device-specific and is not encoded in the request.
The master application must therefore supply the expected length when it
builds the request. The descriptor retains that length for strict response
validation.

```c
static const uint16_t expected_server_id_length = 2u;
mbrtum_request_t request;
uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
size_t request_adu_length = 0u;

int result = mbrtum_build_report_server_id_request(
    1u,
    expected_server_id_length,
    &request,
    request_adu,
    sizeof(request_adu),
    &request_adu_length);
```

Validate a complete response with `mbrtum_process_response()`, then decode it
without copying:

```c
mbrtum_response_t response;
mbrtum_server_id_response_t server_id_response;

result = mbrtum_process_response(
    &request,
    response_adu,
    response_adu_length,
    &response);

if (result == MBRTUM_OK &&
    mbrtum_get_server_id_response(
        &request, &response, &server_id_response) == MBRTUM_OK) {
    /* server_id_response.server_id / server_id_length */
    /* server_id_response.run_status */
    /* additional_data / additional_data_length */
}
```

The validator checks CRC, slave address, function, byte count, exact response
length, maximum size, the expected device-specific Server ID length, and the
required `0x00`/`0xFF` Run Indicator Status at the resulting byte offset.

## Transaction engine

FC11 requests are accepted by `mbrtum_transaction_start()` as ordinary
unicast, response-required transactions. Before transmission, the transaction
engine checks the unicast address, function-only four-byte ADU, expected Server
ID length, zero-valued unused descriptor fields, response expectation, and
CRC. Timeout, retry, exception, malformed-response, and unrelated-response
behavior reuse the existing state machine.

## Memory and hardware boundary

- no heap allocation;
- one fixed 251-byte response-data array;
- copied Server ID and Additional Data;
- no UART, timer, DMA, HAL, or RS-485 dependency;
- board-specific logic decides when the run-status value should be
  reconfigured.

## Verification

The dedicated host suite covers variable-length Server IDs, configuration
guards, copied storage, maximum-size responses, RTU unicast and broadcast
behavior, malformed requests, missing configuration, response capacity, TCP
rejection, master building and decoding, malformed master responses, and
transaction-engine completion.

## Reference

- Modbus Application Protocol Specification V1.1b3, section 6.13, Report
  Server ID.
