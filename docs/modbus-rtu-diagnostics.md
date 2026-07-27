# Portable Modbus RTU diagnostics

This module adds the serial-line-only Modbus functions FC07, FC08, FC0B, and
FC0C without exposing them through the shared PDU dispatcher used by Modbus
TCP.

## Architectural boundary

Ordinary function codes continue to use the shared transport-independent path:

```text
Modbus TCP ADU -> mbtcp_process_adu() -> mb_process_pdu()
Modbus RTU ADU -> mbrtu_process_adu()  -> mb_process_pdu()
```

Diagnostics use a separate RTU-only path:

```text
mbrtu_process_adu_with_diagnostics()
    -> mbrtu_diagnostics_process_pdu() for FC07/FC08/FC0B/FC0C
    -> mb_process_pdu() for ordinary function codes
```

`mbtcp_process_adu()` is unchanged. TCP requests using `0x07`, `0x08`, `0x0B`,
or `0x0C` therefore continue to receive exception `01` (Illegal Function).

## Files

- `App/include/modbus_rtu_diagnostics.h`
- `App/src/modbus_rtu_diagnostics.c`
- `Tests/host/test_modbus_rtu_diagnostics.c`

## Fixed-capacity state

`mbrtu_diagnostics_t` owns all diagnostics data directly:

- exception-status byte
- 16-bit diagnostic register
- 16-bit communication-status word
- communication event counter
- bus message counter
- bus communication-error counter
- bus exception-error counter
- server message counter
- server no-response counter
- server NAK counter
- server busy counter
- bus character-overrun counter
- listen-only state
- pending portable actions
- a 64-byte communication-event log

No heap allocation is used. All 16-bit counters saturate at `0xFFFF`. The
64-byte event log is a ring presented newest first. When full, a new event
replaces the oldest event deterministically. Receive-storage overruns are
recorded in the bus event history; the standard overrun counter increments
only when the truncated frame was addressed to the configured server.

## Build opt-in and backward compatibility

Add `App/src/modbus_rtu_diagnostics.c` and compile both it and
`App/src/modbus_rtu.c` with:

```text
MBRTU_ENABLE_DIAGNOSTICS=1
```

The repository Make and CMake builds do this automatically. If the definition
is omitted, `modbus_rtu.c` uses internal no-op accounting shims, so the original
pre-diagnostics RTU source list still compiles and links without
`modbus_rtu_diagnostics.c`. In that legacy configuration, FC07, FC08, FC0B, and
FC0C remain unsupported by the slave path and produce the same Illegal Function
behavior as before. The master builders and validators remain available from
`modbus_rtu_master.c`.

## Initialization

For complete-frame use:

```c
mbrtu_diagnostics_t diagnostics;
mbrtu_diagnostics_init(&diagnostics);

int result = mbrtu_process_adu_with_diagnostics(
    1u,
    request,
    request_length,
    response,
    sizeof(response),
    &response_length,
    &diagnostics,
    diagnostics_policy,
    policy_context);
```

For the byte/timing server, use `mbrtu_init_with_diagnostics()` instead of
`mbrtu_init()`. The original initialization and complete-frame APIs remain
available and retain their pre-diagnostics behavior.

## Supported slave functions

### FC07 Read Exception Status

The request PDU must contain only function `0x07`. The response returns the
application-owned exception-status byte. Use
`mbrtu_diagnostics_set_exception_status()` to update it.

### FC08 Diagnostics

The implementation supports these serial-line subfunctions:

| Subfunction | Operation | Data rule |
|---:|---|---|
| `0x0000` | Return Query Data | Even number of data bytes, echoed exactly |
| `0x0001` | Restart Communications Option | `0x0000` or `0xFF00` |
| `0x0002` | Return Diagnostic Register | Request data `0x0000` |
| `0x0004` | Force Listen Only Mode | Request data `0x0000` |
| `0x000A` | Clear Counters and Diagnostic Register | Request data `0x0000` |
| `0x000B` | Return Bus Message Count | Request data `0x0000` |
| `0x000C` | Return Bus Communication Error Count | Request data `0x0000` |
| `0x000D` | Return Bus Exception Error Count | Request data `0x0000` |
| `0x000E` | Return Server Message Count | Request data `0x0000` |
| `0x000F` | Return Server No Response Count | Request data `0x0000` |
| `0x0010` | Return Server NAK Count | Request data `0x0000` |
| `0x0011` | Return Server Busy Count | Request data `0x0000` |
| `0x0012` | Return Bus Character Overrun Count | Request data `0x0000` |
| `0x0014` | Clear Overrun Counter and Flag | Request data `0x0000` |

The ASCII delimiter subfunction `0x0003` is not implemented by this RTU-only
module. Reserved or unsupported subfunctions return exception `01`. Invalid
lengths or data values return exception `03` for unicast requests.

### FC0B Get Communication Event Counter

The request PDU must contain only function `0x0B`. The response contains the
communication-status word and event counter.

### FC0C Get Communication Event Log

The request PDU must contain only function `0x0C`. The response contains:

- byte count
- communication-status word
- event counter
- message counter
- zero to 64 newest-first event bytes

## State-changing policy

State-changing FC08 operations are denied unless the caller supplies a policy
callback that returns nonzero. This protects restart, listen-only, counter
clear, and overrun-clear operations from accidental application exposure.

A denied unicast operation returns exception `03`. A broadcast operation never
returns a response.

```c
static int diagnostics_policy(void *context,
                              mbrtu_diagnostics_action_t action,
                              uint16_t data)
{
    (void)context;
    (void)data;

    switch (action) {
    case MBRTU_DIAGNOSTICS_ACTION_CLEAR_COUNTERS_AND_REGISTER:
    case MBRTU_DIAGNOSTICS_ACTION_CLEAR_OVERRUN_COUNTER:
        return 1;
    default:
        return 0;
    }
}
```

## Listen-only and hardware actions

Force Listen Only Mode produces no response. While listen-only is active, all
ordinary requests and all diagnostics except Restart Communications Option are
monitored but not executed and receive no response.

Restart Communications Option is the only request that exits listen-only. When
received while listen-only, it also produces no response.

The portable core does not restart UART hardware or alter board-specific
communications controls. Instead it records action bits. The application can
retrieve and clear them with:

```c
uint8_t actions =
    mbrtu_diagnostics_take_pending_actions(&diagnostics);

if ((actions & MBRTU_DIAGNOSTICS_PENDING_RESTART) != 0u) {
    /* Restart UART/timer state after any required response has transmitted. */
}
```

This keeps STM32 HAL, UART restart, and RS-485 DE/RE control outside the
portable protocol core.

## Broadcast rules

- FC07, FC0B, FC0C, and read-only FC08 subfunctions are ignored when broadcast.
- FC08 `0x0001`, `0x0004`, `0x000A`, and `0x0014` may be broadcast.
- State-changing broadcasts still require policy approval.
- Broadcasts never generate a response.

## Master API

New builders:

- `mbrtum_build_read_exception_status_request()`
- `mbrtum_build_diagnostics_request()`
- `mbrtum_build_get_comm_event_counter_request()`
- `mbrtum_build_get_comm_event_log_request()`

FC08 validation requires the original request ADU so echoed data can be checked
byte-for-byte:

```c
int result = mbrtum_process_response_with_request_adu(
    &request,
    request_adu,
    request_adu_length,
    response_adu,
    response_adu_length,
    &response);
```

The original `mbrtum_process_response()` remains unchanged for ordinary
functions and can validate FC07, FC0B, and FC0C. It returns
`MBRTUM_ERROR_REQUEST_DATA_REQUIRED` for FC08.

Zero-copy decoder helpers are provided for exception status, arbitrary FC08
data (`mbrtum_get_diagnostics_response()`), diagnostic words, communication
event counter responses, and communication event log responses.
The asynchronous transaction engine uses the request-ADU-aware validator and
therefore supports all four diagnostics function codes.

## Verification

The dedicated host test covers:

- normal and malformed FC07, FC08, FC0B, and FC0C requests
- diagnostics opt-in behavior of the legacy RTU API
- strict compile/link verification of the pre-diagnostics RTU source list
- byte/timing-layer dispatch and addressed-overrun accounting
- policy approval and denial
- listen-only entry, suppression, and restart recovery
- broadcast rules
- reset behavior and pending actions
- counter and event-log behavior
- deterministic 64-event overflow
- master builders and strict response validation
- zero-copy decoders
- transaction-engine use
- explicit Modbus TCP Illegal Function responses for all four serial-only codes

Board-specific UART restart, physical RS-485 control, and STM32 hardware
validation are intentionally outside this milestone.
