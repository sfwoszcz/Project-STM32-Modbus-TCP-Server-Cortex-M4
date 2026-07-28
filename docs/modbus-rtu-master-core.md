# Portable Modbus RTU master complete-frame core

This stage adds a portable Modbus RTU master request and response core without
coupling it to an MCU, UART driver, timer, DMA engine, RTOS, scheduler, or
physical transceiver.

The core builds complete RTU request Application Data Units (ADUs), records the
request metadata required to validate a reply, validates one complete response
ADU, and exposes zero-copy helpers for decoded bit and register data.

## Scope

Implemented now:

- request generation for function codes `01`, `02`, `03`, `04`, `05`, `06`,
  `0F`, `10`, and `17`
- Modbus CRC-16 generation in low-byte-first RTU wire order
- unicast slave addresses from 1 through 247
- broadcast writes at address 0
- request quantity, value, address-range, and output-capacity validation
- complete response length and CRC validation
- slave-address and function-code matching
- Modbus exception-response decoding
- exact read byte-count validation
- zero-valued unused padding validation for packed-bit responses
- exact single-write address/value acknowledgement validation
- exact multiple-write address/quantity acknowledgement validation
- zero-copy bit, register, FC20 subresponse, and FC43/14 object access helpers
- strict host tests and integration with Make, CMake, and CTest
- no dynamic allocation

Not implemented in this stage:

- UART transmission or reception
- byte-by-byte response framing
- T1.5 or T3.5 timing for master reception
- response deadlines
- retry policy
- transaction scheduling or queuing
- STM32 HAL or CubeMX integration
- RS-485 driver-enable or receiver-enable control
- physical master hardware validation

Those responsibilities belong to later transaction and transport layers.

## Layer boundary

The complete-frame master core sits between application code and a serial
transport:

```text
application request
        |
        v
mbrtum_build_*_request()
        |
        | complete request ADU
        v
UART / RS-485 transport
        |
        | complete response ADU
        v
mbrtum_process_response()
        |
        v
validated response view and decoding helpers
```

The transport owns transmission, receive timing, timeout detection, and frame
collection. The master core owns Modbus request encoding and response
validation.

## Supported function codes

| Function | Name | Request limit | Normal response |
|---:|---|---:|---|
| `01` | Read Coils | 1 to 2000 bits | packed bits |
| `02` | Read Discrete Inputs | 1 to 2000 bits | packed bits |
| `03` | Read Holding Registers | 1 to 125 registers | big-endian registers |
| `04` | Read Input Registers | 1 to 125 registers | big-endian registers |
| `05` | Write Single Coil | one coil | echoed address and encoded value |
| `06` | Write Single Holding Register | one register | echoed address and value |
| `0F` | Write Multiple Coils | 1 to 1968 coils | echoed address and quantity |
| `10` | Write Multiple Holding Registers | 1 to 123 registers | echoed address and quantity |
| `14` | Read File Record | 35 subrequests; combined response data <= 245 bytes | file subresponses |
| `17` | Read/Write Multiple Registers | read 1 to 125; write 1 to 121 | big-endian read registers |
| `2B/0E` | Read Device Identification | Basic/Regular/Extended/Specific | segmented object list |

The request builders reject an address range whose final item would exceed
Modbus address `65535`.

## Public files

```text
App/include/modbus_rtu_master.h
App/src/modbus_rtu_master.c
Tests/host/test_modbus_rtu_master.c
```

The public API uses the `mbrtum_` prefix so that master functions remain
distinct from the existing `mbrtu_` slave/server API.

## Request descriptor

Each successful builder populates an `mbrtum_request_t`:

```c
typedef struct {
    uint8_t slave_address;
    uint8_t function;
    uint16_t start_address;
    uint16_t quantity;
    uint16_t value;
    uint8_t expects_response;
    uint16_t write_start_address;
    uint16_t write_quantity;
} mbrtum_request_t;
```

The descriptor must remain unchanged until the corresponding response has been
validated.

`value` contains:

- `0xFF00` or `0x0000` for FC05
- the written register value for FC06
- zero for the other supported functions

`expects_response` is zero only for a valid broadcast write.

## Response view

A validated response is represented by `mbrtum_response_t`:

```c
typedef struct {
    uint8_t function;
    uint8_t exception_code;
    const uint8_t *data;
    size_t data_length;
} mbrtum_response_t;
```

For normal FC01 through FC04 replies, `data` points directly into the
caller-owned response ADU after the byte-count field. No copy or allocation is
performed.

For successful write acknowledgements and Modbus exception responses, `data`
is `NULL` and `data_length` is zero.

The response ADU must remain unchanged while the view is being used.

## Build a read request

Example FC03 request:

```c
#include "modbus_rtu_master.h"

mbrtum_request_t request;
uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
size_t request_adu_length = 0u;

int result = mbrtum_build_read_registers_request(
    1u,
    MBRTUM_FC_READ_HOLDING_REGISTERS,
    100u,
    4u,
    &request,
    request_adu,
    sizeof(request_adu),
    &request_adu_length);
```

On success, `request_adu` contains:

```text
slave address | function | start address | quantity | CRC low | CRC high
```

The caller then transmits exactly `request_adu_length` bytes.

## Validate and decode a read response

After the transport has collected one complete response ADU:

```c
mbrtum_response_t response;

result = mbrtum_process_response(
    &request,
    response_adu,
    response_adu_length,
    &response);

if (result == MBRTUM_OK) {
    uint16_t value;

    if (mbrtum_get_register(&request, &response, 0u, &value) == MBRTUM_OK) {
        /* Use the first returned register. */
    }
} else if (result == MBRTUM_EXCEPTION_RESPONSE) {
    uint8_t remote_exception = response.exception_code;
    /* Handle the remote Modbus exception. */
}
```

The response validator checks the original request descriptor before exposing
response data.

## Write multiple registers

Example FC10 request:

```c
const uint16_t values[] = {
    0x1111u,
    0x2222u,
    0x3333u
};

mbrtum_request_t request;
uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
size_t request_adu_length = 0u;

int result = mbrtum_build_write_multiple_registers_request(
    1u,
    10u,
    3u,
    values,
    &request,
    request_adu,
    sizeof(request_adu),
    &request_adu_length);
```

A normal FC10 response is accepted only when its echoed start address and
quantity exactly match the generated request descriptor.

## Packed coils

FC0F input values and FC01/FC02 response values use Modbus least-significant-bit
first packing.

For ten bits, two data bytes are required:

```text
byte 0: bits 0 through 7
byte 1: bits 8 and 9 in positions 0 and 1
```

The FC0F builder clears unused high bits in the final request byte.

For FC01 and FC02 responses, the validator rejects a frame when unused high
bits in the final packed byte are nonzero. This strict check prevents malformed
packed-bit data from being accepted.

## FC20 requests

`mbrtum_build_read_file_record_request()` accepts up to 35 fixed-size
subrequest descriptors. It validates file number, record range, request-data
length, combined expected response-data length, output capacity, and CRC. The
request descriptor stores the request byte count, expected response byte
count, and subrequest count.

FC20 response validation requires
`mbrtum_process_response_with_request_adu()` so every returned subresponse
length can be checked against its original requested register count.
`mbrtum_get_file_record_response()`,
`mbrtum_get_file_record_subresponse()`, and
`mbrtum_get_file_record_register()` provide bounded zero-copy decoding. See
[`modbus-fc20-read-file-record.md`](modbus-fc20-read-file-record.md).

## FC23 requests

`mbrtum_build_read_write_multiple_registers_request()` records the read range in
`start_address` and `quantity`, and the write range in
`write_start_address` and `write_quantity`. The builder accepts only unicast
addresses, encodes up to 121 write values, and can generate a maximum 255-byte
RTU ADU. The normal response is decoded through `mbrtum_get_register()` using
the requested read quantity. See [`modbus-fc23.md`](modbus-fc23.md).

## Broadcast writes

Address zero is accepted only by the write-only builders:

- FC05
- FC06
- FC0F
- FC10

A successful broadcast builder sets:

```c
request.expects_response = 0u;
```

The transport must transmit the request and must not wait for a response.
Calling `mbrtum_process_response()` with a broadcast request returns
`MBRTUM_ERROR_RESPONSE_NOT_EXPECTED`.

Read builders, including FC20 and FC23, reject address zero.

## Modbus exception responses

A remote exception response has this RTU PDU form:

```text
requested function | 0x80
exception code
```

The complete RTU exception ADU contains:

```text
slave address | exception function | exception code | CRC low | CRC high
```

A valid exception response causes `mbrtum_process_response()` to return
`MBRTUM_EXCEPTION_RESPONSE`. The exception code is available through
`response.exception_code`.

An exception code of zero is treated as malformed.

## Validation order

`mbrtum_process_response()` performs these checks:

1. API arguments and request descriptor
2. whether the request expects a response
3. complete ADU size
4. CRC
5. slave-address match
6. normal or exception function match
7. function-specific response length
8. read byte count or write acknowledgement fields
9. packed-bit padding where applicable

The response view is cleared before validation, so error paths never leave stale
data pointers or stale exception information.

## Return values

Successful results:

```text
MBRTUM_OK
MBRTUM_EXCEPTION_RESPONSE
```

Error categories include:

```text
MBRTUM_ERROR_ARGUMENT
MBRTUM_ERROR_SLAVE_ADDRESS
MBRTUM_ERROR_FUNCTION
MBRTUM_ERROR_QUANTITY
MBRTUM_ERROR_VALUE
MBRTUM_ERROR_CAPACITY
MBRTUM_ERROR_RESPONSE_LENGTH
MBRTUM_ERROR_CRC
MBRTUM_ERROR_ADDRESS_MISMATCH
MBRTUM_ERROR_FUNCTION_MISMATCH
MBRTUM_ERROR_MALFORMED_RESPONSE
MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH
MBRTUM_ERROR_RESPONSE_NOT_EXPECTED
MBRTUM_ERROR_INDEX
```

A Modbus exception returned by a remote slave is not a local parser failure. It
is reported separately as `MBRTUM_EXCEPTION_RESPONSE`.

## Storage and ownership

The core uses caller-owned storage and performs no dynamic allocation.

The following objects must not overlap:

- request descriptor
- request ADU buffer
- request ADU length object
- FC0F packed-value input
- FC10 and FC23 register-value input
- FC20 subrequest descriptor input

For response processing, these objects must not overlap:

- request descriptor
- response ADU
- response view

The response view references the response ADU and is valid only while that ADU
remains unchanged.

## Host verification

`Tests/host/test_modbus_rtu_master.c`,
`Tests/host/test_modbus_fc20.c`, and
`Tests/host/test_modbus_fc23.c` verify:

- all ordinary supported request builders, including FC20 and FC23
- known request field layouts and generated CRCs
- unicast and broadcast behavior
- maximum FC0F, FC10, FC20, and FC23 request sizes
- output-capacity failures
- address-range overflow rejection
- normal bit, register, FC20 subresponse, and FC23 response decoding
- index validation
- Modbus exception responses
- invalid exception code and exception length rejection
- CRC corruption rejection
- slave-address mismatch rejection
- function mismatch rejection
- malformed read byte counts
- nonzero packed-bit padding rejection
- single-write acknowledgement mismatch rejection
- multiple-write acknowledgement mismatch rejection
- API argument validation

Run the complete verification suite with:

```bash
make clean && make test
```

Or use CMake and CTest:

```bash
cmake -S . -B build
cmake --build build --clean-first
ctest --test-dir build --output-on-failure
```

The CTest suite includes dedicated `rtu_master`, `fc20`, `fc23`, and
`fc43_device_id` tests.

## Transport boundary

The complete-frame core intentionally excludes UART handling, response timers,
retry scheduling, and RS-485 direction control. The portable transaction engine
adds request ownership, deadlines, retries, and transport events above this
layer. STM32 UART reception, timer integration, and RS-485 DE/RE control remain
in board-specific adapters.

## FC43/14 Device Identification

`mbrtum_build_read_device_identification_request()` builds the fixed seven-byte
RTU request ADU for MEI type `0x0E`. The request descriptor stores the Read
Device ID code in `start_address`, the object ID in `quantity`, and the MEI type
in `value`. `mbrtum_process_response()` validates conformity, segmentation,
object boundaries, ordering, category, and Specific Object acknowledgement.
`mbrtum_get_device_id_response()` and `mbrtum_get_device_id_object()` expose
bounded zero-copy views into the validated response ADU. See
`docs/modbus-fc43-device-identification.md`.
