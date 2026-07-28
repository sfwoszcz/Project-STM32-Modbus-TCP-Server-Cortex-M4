# Modbus FC20 Read File Record

This document describes the portable implementation of Modbus function code
`0x14` (Read File Record).

FC20 is a shared application-protocol function. The same PDU implementation is
used by Modbus TCP and Modbus RTU.

## Wire format

The request PDU contains a one-byte request-data length followed by one or more
seven-byte subrequests:

```text
Function 0x14
Request data length
  Reference type 0x06
  File number        2 bytes, big-endian
  Record number      2 bytes, big-endian
  Record length      2 bytes, big-endian registers
  ...additional seven-byte subrequests...
```

The request-data length must be from 7 through 245 bytes and must be divisible
by seven. A request can therefore contain at most 35 subrequests.

The normal response contains one subresponse per subrequest:

```text
Function 0x14
Response data length
  File response length
  Reference type 0x06
  Record data         2 bytes per register, big-endian
  ...additional subresponses...
```

`File response length` includes the reference-type byte and record-data bytes,
but not the file-response-length byte itself.

## Address rules

The portable core enforces:

- reference type exactly `0x06`;
- file number from 1 through 65535;
- record number from 0 through 9999;
- non-zero record length;
- `record number + record length <= 10000`;
- the requested range must fit the configured application file;
- the combined response data must not exceed 245 bytes.

File numbers above 10 are accepted because the protocol permits them, although
legacy-device interoperability can be lower for those file numbers.

## Application file map

Applications configure up to `MB_FILE_RECORD_MAX_FILES` descriptors:

```c
#include "modbus.h"
#include "modbus_file_record.h"

static uint16_t recipe_file[128];
static uint16_t history_file[1000];

static const mb_file_record_file_t files[] = {
    {1u, recipe_file, 128u},
    {2u, history_file, 1000u}
};

int application_modbus_init(void)
{
    mb_init();
    return mb_file_record_configure(
        files,
        sizeof(files) / sizeof(files[0]));
}
```

The configuration function copies the fixed-capacity descriptors, not the
record arrays. Application record arrays must remain valid while configured.
This avoids heap allocation and avoids reserving a fixed 10000-register array
inside the portable core for every possible file.

Descriptors must be sorted by strictly increasing file number. Failed
configuration leaves the previous map unchanged. `mb_init()` and
`mb_file_record_clear()` clear the configured map.

The record pointer is writable so the same map can be extended later for FC21
Write File Record without replacing the public storage model.

## Slave/server processing

`mb_process_pdu()` dispatches FC20 to the fixed-capacity file-record map.
Therefore:

- `mbtcp_process_adu()` exposes FC20 through Modbus TCP;
- `mbrtu_process_adu()` exposes FC20 through Modbus RTU;
- transport wrappers require no FC20-specific changes.

The implementation validates all subrequests and the complete response size
before copying any record data. Invalid request lengths and quantities return
Illegal Data Value. Invalid reference types, file numbers, record numbers, or
configured ranges return Illegal Data Address. Insufficient caller response
capacity returns Server Device Failure.

## RTU master builder

`mbrtum_build_read_file_record_request()` accepts an array of
`mbrtum_file_record_request_t` values:

```c
static const mbrtum_file_record_request_t reads[] = {
    {4u, 1u, 2u},
    {3u, 9u, 2u}
};

mbrtum_request_t request;
uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
size_t request_adu_length;

int result = mbrtum_build_read_file_record_request(
    1u,
    reads,
    sizeof(reads) / sizeof(reads[0]),
    &request,
    request_adu,
    sizeof(request_adu),
    &request_adu_length);
```

The builder validates every subrequest, the 245-byte request-data limit, the
combined expected response-data limit, output capacity, slave address, and
complete CRC.

The request descriptor records:

- `start_address`: request-data byte count;
- `quantity`: expected response-data byte count;
- `value`: subrequest count.

## Master response validation

FC20 response validation requires the immutable original request ADU so each
subresponse can be compared with its corresponding requested record length:

```c
mbrtum_response_t response;

int result = mbrtum_process_response_with_request_adu(
    &request,
    request_adu,
    request_adu_length,
    response_adu,
    response_adu_length,
    &response);
```

Calling `mbrtum_process_response()` without the original request ADU returns
`MBRTUM_ERROR_REQUEST_DATA_REQUIRED` for FC20.

The validator checks:

- complete RTU length and CRC;
- slave and function matching;
- response data byte count;
- exact number and order of subresponses;
- each file-response length against its requested record length;
- reference type `0x06`;
- exact end of frame with no omitted or trailing data.

## Zero-copy decoding

A validated response can be decoded without copying record data:

```c
mbrtum_file_record_response_t file_response;
mbrtum_file_record_subresponse_t subresponse;
uint16_t value;

mbrtum_get_file_record_response(&response, &file_response);
mbrtum_get_file_record_subresponse(&file_response, 0u, &subresponse);
mbrtum_get_file_record_register(&subresponse, 0u, &value);
```

All views point into the caller-owned response ADU and remain valid only while
that ADU remains unchanged.

## Transaction engine

The portable transaction engine accepts FC20 requests produced by the builder.
Before transmission it verifies the descriptor, every request subfield, exact
ADU length, response-size metadata, and CRC. Normal responses, Modbus
exceptions, timeouts, retries, unrelated frames, and malformed-frame accounting
reuse the existing transaction state machine.

## Broadcast behavior

FC20 is a read function. The master builder rejects slave address zero. The RTU
slave wrapper ignores broadcast read requests and sends no response.

## Verification

`Tests/host/test_modbus_fc20.c` covers:

- file-map configuration and failed-update preservation;
- application-owned record data;
- the two-subrequest protocol example;
- shared PDU, TCP, and RTU processing;
- malformed request lengths and byte counts;
- reference, file, record, range, and zero-length failures;
- maximum 35-subrequest requests;
- maximum 121-register single subresponses;
- response-capacity failures;
- master builder fields and CRC;
- request-ADU-aware response validation;
- exception responses and malformed subresponses;
- zero-copy subresponse and register decoding;
- master transaction-engine completion and request guards.
