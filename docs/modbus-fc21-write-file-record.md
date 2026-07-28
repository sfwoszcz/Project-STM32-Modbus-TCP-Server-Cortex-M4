# Modbus FC21 Write File Record

This module adds portable support for Modbus function code 21 (`0x15`), Write
File Record, to the shared transport-independent PDU engine and the complete-
frame Modbus RTU master core.

The implementation follows the Modbus Application Protocol Specification
V1.1b3 and keeps the project constraints already used by FC20:

- no heap allocation;
- fixed-capacity descriptor storage;
- application-owned file data;
- strict complete-request validation;
- deterministic response generation;
- shared Modbus TCP and Modbus RTU slave processing;
- host-testable code without UART or STM32 HAL dependencies.

## File-record storage

FC21 uses the same application-configured file map as FC20. Include:

```c
#include "modbus_file_record.h"
```

One configured file is described by:

```c
typedef struct {
    uint16_t file_number;
    uint16_t *records;
    size_t record_count;
} mb_file_record_file_t;
```

The application owns each `records` array. The portable core copies only the
file descriptors and writes directly into the supplied arrays while holding the
project Modbus lock.

Configuration rules remain:

- 1 to `MB_FILE_RECORD_MAX_FILES` files;
- file numbers from 1 to 65535;
- 1 to 10000 registers per configured file;
- descriptors sorted by strictly increasing file number;
- record storage remains valid while configured.

Example:

```c
static uint16_t recipe_file[10000];
static uint16_t calibration_file[256];

static const mb_file_record_file_t files[] = {
    { 1u, recipe_file, 10000u },
    { 4u, calibration_file, 256u }
};

int result = mb_file_record_configure(files,
                                      sizeof(files) / sizeof(files[0]));
```

`mb_init()` and `mb_file_record_clear()` remove the descriptors but do not
modify the application-owned arrays.

## Request format

The FC21 request PDU is:

```text
function 0x15
request data length
subrequest 1
subrequest 2
...
```

Each variable-length subrequest contains:

```text
reference type       1 byte, must be 0x06
file number          2 bytes, big-endian, 1..65535
record number        2 bytes, big-endian, 0..9999
record length        2 bytes, big-endian, number of registers
record data          record length * 2 bytes, big-endian
```

The request-data length is 9 to 251 bytes. The minimum is one subrequest with
one register. The maximum allows a complete 253-byte Modbus PDU and a complete
256-byte Modbus RTU ADU.

The fixed-capacity implementation defines:

```c
MB_FILE_RECORD_WRITE_MAX_SUBREQUESTS
MB_FILE_RECORD_WRITE_REQUEST_DATA_MAX
MB_FILE_RECORD_WRITE_RESPONSE_DATA_MAX
```

The maximum number of one-register subrequests is 27. A request may instead use
fewer subrequests with longer record ranges, provided the combined request data
does not exceed 251 bytes.

## Slave processing

FC21 is dispatched by `mb_process_pdu()`. Therefore the same implementation is
available through:

- `mbtcp_process_adu()` for Modbus TCP;
- `mbrtu_process_adu()` and `mbrtu_process_adu_with_diagnostics()` for Modbus
  RTU.

The slave validates the entire request before modifying any file data. The
validation pass checks:

- exact request-data length and exact PDU length;
- complete seven-byte subrequest headers;
- reference type `0x06`;
- non-zero file number;
- non-zero record length;
- record number and end-of-range within 0..9999;
- configured file existence;
- range availability inside the configured application array;
- complete record-data bytes for every subrequest;
- output capacity for the exact normal response echo.

If any subrequest is invalid, no earlier subrequest is written. This gives the
request deterministic all-or-nothing validation behavior at the protocol
layer.

After validation succeeds, subrequests are applied in request order. If two
subrequests overlap, later data in the request overwrites earlier data in the
overlapping registers.

The normal response is an exact echo of the request PDU and is generated only
after all writes complete.

## Exceptions

The shared PDU layer returns standard Modbus exceptions:

- Illegal Data Address (`0x02`) for an unsupported reference type, unknown file,
  invalid record address, or configured-file range failure;
- Illegal Data Value (`0x03`) for malformed lengths, zero record length,
  incomplete headers, or incomplete record data;
- Server Device Failure (`0x04`) when the caller-provided response buffer cannot
  hold the exact echo.

No write is applied when request validation or response-capacity validation
fails.

## Modbus RTU broadcast

FC21 is a write-only operation and is accepted at RTU broadcast address zero.
The shared PDU request is fully validated and applied, but the RTU wrapper
suppresses the response as required for broadcast requests.

Invalid broadcast FC21 requests also produce no wire response and do not apply
partial writes.

Modbus TCP has no broadcast-address behavior; FC21 uses the normal request and
response transaction.

## RTU master request builder

The master API describes one write subrequest with:

```c
typedef struct {
    uint16_t file_number;
    uint16_t record_number;
    uint16_t record_length;
    const uint16_t *record_data;
} mbrtum_write_file_record_request_t;
```

Build a complete RTU request with:

```c
int mbrtum_build_write_file_record_request(
    uint8_t slave_address,
    const mbrtum_write_file_record_request_t *subrequests,
    size_t subrequest_count,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);
```

The builder:

- accepts unicast addresses 1..247 and broadcast address 0;
- validates every file, record, length, and data pointer;
- validates the complete 251-byte request-data limit;
- encodes host-endian `uint16_t` values in Modbus big-endian order;
- appends the low-byte-first Modbus RTU CRC;
- sets `request.expects_response` to zero for broadcast;
- performs no dynamic allocation.

For FC21 the request descriptor stores:

```text
start_address  = request-data byte count
quantity       = subrequest count
value          = 0
```

The transaction-owned request ADU retains the complete variable-length
subrequest data needed for exact response validation.

## Master response validation

A successful FC21 response must echo the complete request ADU fields exactly.
Because the compact request descriptor cannot store all variable-length record
data, normal FC21 validation requires:

```c
mbrtum_process_response_with_request_adu(...)
```

The validator checks:

- request descriptor consistency;
- original request ADU length, function, byte count, subrequests, and CRC;
- response length and CRC;
- slave address and function;
- byte-for-byte equality with the original request ADU.

A valid Modbus exception response is decoded normally. Calling
`mbrtum_process_response()` without the original request ADU returns
`MBRTUM_ERROR_REQUEST_DATA_REQUIRED` for a normal FC21 response.

Broadcast request descriptors return `MBRTUM_ERROR_RESPONSE_NOT_EXPECTED` if a
response is submitted.

## Transaction engine

The portable master transaction engine accepts FC21 requests built by
`mbrtum_build_write_file_record_request()`.

Before transmission it revalidates:

- slave address and response expectation;
- request-data length and subrequest count;
- each reference type, file number, record number, and record length;
- each variable record-data extent;
- exact ADU length and CRC;
- consistency with the immutable request descriptor.

For unicast requests, the engine waits for and validates the exact response
echo through its transaction-owned request ADU. For broadcast requests, the
transaction completes after successful transmit completion and never starts a
response deadline.

## Threading and ownership

The configured file descriptors are copied into fixed internal storage. The
record arrays remain application-owned.

The core accesses file descriptors and record data while holding `mb_lock()`.
Projects that override the lock hooks must ensure that application code which
accesses the same arrays follows a compatible synchronization policy.

Input arrays supplied to the master builder must remain valid only for the
duration of the builder call. The generated request ADU owns the encoded wire
data afterward.

## Host verification

`Tests/host/test_modbus_fc21.c` covers:

- the specification request/response echo example;
- shared-PDU writes and exact echoing;
- complete-request atomic validation;
- malformed byte counts and truncated record data;
- invalid reference, file, record, and length fields;
- overlapping subrequests and request-order behavior;
- maximum 253-byte PDU and 256-byte RTU ADU;
- Modbus TCP processing;
- RTU unicast and broadcast processing;
- master builder field layout and CRC;
- unicast and broadcast descriptors;
- maximum-size and capacity checks;
- exact master response-echo validation;
- Modbus exception responses;
- request-ADU tampering rejection;
- master transaction completion for unicast and broadcast.
