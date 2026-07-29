# FC43 Encapsulated Interface Transport

This module adds a bounded, transport-independent implementation of Modbus
function code 43 (`0x2B`). FC43 carries an 8-bit Modbus Encapsulated Interface
(MEI) type followed by interface-specific data.

The project includes MEI type `0x0E` Read Device Identification as a built-in
strictly validated interface. MEI type `0x0D` CANopen General Reference is
intentionally not implemented. Other MEI types have no behavior until the
application explicitly registers a handler.

## PDU format

Generic request and response PDUs use the same envelope:

```text
+---------------+----------+---------------------------+
| Function 0x2B | MEI type | Interface-specific bytes  |
+---------------+----------+---------------------------+
      1 byte       1 byte          0 to 251 bytes
```

The complete PDU is bounded by the normal Modbus maximum of 253 bytes. The
library echoes the request MEI type in a successful response. The application
handler reads and writes only the interface-specific bytes.

## Server registration API

```c
#include "modbus_mei.h"

static uint8_t my_mei_handler(void *context,
                              uint8_t mei_type,
                              const uint8_t *request_data,
                              size_t request_data_length,
                              uint8_t *response_data,
                              size_t response_capacity,
                              size_t *response_data_length);

int result = mb_mei_register_handler(assigned_mei_type,
                                     my_mei_handler,
                                     application_context);
```

The fixed-capacity registry stores at most `MB_MEI_MAX_HANDLERS` entries, eight
by default. Registering an existing type atomically replaces its callback and
context. `mb_mei_unregister_handler()` removes one entry and
`mb_mei_clear_handlers()` removes all generic entries. `mb_init()` also clears
the generic registry.

Callbacks are invoked without the library lock held. The callback and its
context must remain valid until unregistered, and application synchronization
is responsible for mutable context data.

The callback returns zero for success or a standard Modbus exception code. On
success it sets the interface-specific response length. An invalid exception
code, an oversized reported response, or insufficient output capacity becomes
Server Device Failure (`0x04`). An unregistered MEI type and the intentionally
unsupported CANopen type return Illegal Data Value (`0x03`).

## Assigned-type boundary

The framework is generic, but it does not assign an MEI type. Applications must
use only an MEI type assigned or otherwise permitted for their deployment.
Reserved values are disabled by default because no handler is registered.

Two types are protected by the core:

- `0x0D`: CANopen General Reference, rejected because CANopen is outside scope;
- `0x0E`: Read Device Identification, handled by the existing built-in FC43/14
  implementation and not replaceable through the generic registry.

## Shared TCP and RTU paths

`mb_process_pdu()` dispatches every FC43 request through the generic envelope.
MEI `0x0E` is delegated to the existing Device Identification object model;
registered generic types are delegated to the application callback. The same
PDU path is reused by both `mbtcp_process_adu()` and `mbrtu_process_adu()`.
Neither transport wrapper is modified.

RTU address-zero frames do not generate responses. Applications should not use
broadcasts for request/response MEI services.

## RTU master API

```c
int mbrtum_build_mei_request(uint8_t slave_address,
                             uint8_t mei_type,
                             const uint8_t *mei_data,
                             size_t mei_data_length,
                             mbrtum_request_t *request,
                             uint8_t *request_adu,
                             size_t request_adu_capacity,
                             size_t *request_adu_length);
```

The builder supports zero through 251 MEI-data bytes and produces a complete
CRC-protected RTU ADU. It rejects broadcast addresses, CANopen type `0x0D`, and
Device Identification type `0x0E`. FC43/14 continues to use
`mbrtum_build_read_device_identification_request()` for strict interface-
specific validation.

A generic validated response is decoded without copying:

```c
mbrtum_mei_response_t view;
int result = mbrtum_get_mei_response(&response, &view);
```

`view.mei_type` is the echoed type and `view.data` references the remaining
interface bytes in the caller-owned response ADU. Generic validation checks the
complete ADU length, CRC, slave address, function, and echoed MEI type. Semantic
validation of interface data belongs to the interface implementation.

The portable transaction engine accepts generic builder output, verifies the
immutable descriptor, exact request length, MEI type, and CRC, and then reuses
the existing timeout, retry, exception, unrelated-frame, and malformed-frame
policies.

## Compatibility

- no heap allocation;
- fixed-capacity handler registry;
- no changes to the TCP or RTU transport wrapper APIs;
- existing FC43/14 public behavior and strict validation preserved;
- legacy RTU source-list linking preserved because the registry and dispatcher
  are integrated into existing core sources;
- handlers are unregistered by default.

## Test coverage

`Tests/host/test_modbus_fc43_mei.c` covers:

- registration, replacement, removal, registry capacity, and `mb_init()` reset;
- CANopen and FC43/14 registration protection;
- unregistered, malformed, callback-exception, invalid-return, capacity, and
  oversized-response failures;
- shared PDU, Modbus TCP, Modbus RTU, CRC, and maximum 253-byte PDU paths;
- preservation of the built-in FC43/14 Device Identification behavior;
- generic RTU master request generation and zero-copy response decoding;
- echoed-MEI mismatch, CRC, and Modbus exception validation;
- transaction-engine request validation and successful completion.
