# FC43/14 Read Device Identification

This milestone adds portable support for Modbus function code 43 (`0x2B`) with
MEI type 14 (`0x0E`), Read Device Identification.

FC43/14 is a shared Modbus application function and is available through both
server transports:

```text
Modbus TCP ADU -> mbtcp_process_adu() -> mb_process_pdu()
Modbus RTU ADU -> mbrtu_process_adu()  -> mb_process_pdu()
```

The implementation also adds complete-frame Modbus RTU master request,
response-validation, zero-copy decoding, and transaction-engine support.

## Device Identification object model

The interface is an ordered collection of byte-string objects. Standard object
IDs are:

| Object ID | Name | Category |
|---:|---|---|
| `0x00` | VendorName | mandatory Basic |
| `0x01` | ProductCode | mandatory Basic |
| `0x02` | MajorMinorRevision | mandatory Basic |
| `0x03` | VendorUrl | optional Regular |
| `0x04` | ProductName | optional Regular |
| `0x05` | ModelName | optional Regular |
| `0x06` | UserApplicationName | optional Regular |
| `0x80`-`0xFF` | product-specific objects | optional Extended |

IDs `0x07` through `0x7F` are reserved and are rejected by the configuration
API.

The portable state uses no heap allocation. `mb_device_id_configure()` validates
and copies the caller's object values into fixed-capacity library storage:

- up to `MB_DEVICE_ID_MAX_OBJECTS` objects, default 32;
- up to `MB_DEVICE_ID_VALUE_STORAGE_SIZE` total value bytes, default 512;
- up to `MB_DEVICE_ID_MAX_VALUE_LENGTH` bytes per object, fixed at 244 so one
  object always fits in a maximum 253-byte Modbus PDU.

The mandatory Basic objects must be the first three entries. All entries must be
strictly ordered by object ID. The complete candidate configuration is
validated before the active configuration is replaced.

`mb_init()` clears the object model, so applications configure device
identification after initializing the protocol core.

Example:

```c
#include "modbus.h"
#include "modbus_device_id.h"

static const uint8_t vendor[] = "Example Vendor";
static const uint8_t product[] = "STM32-MODBUS";
static const uint8_t revision[] = "1.0";

static const mb_device_id_object_t objects[] = {
    {MB_DEVICE_ID_OBJECT_VENDOR_NAME, vendor, sizeof(vendor) - 1u},
    {MB_DEVICE_ID_OBJECT_PRODUCT_CODE, product, sizeof(product) - 1u},
    {MB_DEVICE_ID_OBJECT_MAJOR_MINOR_REVISION,
     revision,
     sizeof(revision) - 1u}
};

mb_init();
(void)mb_device_id_configure(
    MB_DEVICE_ID_CONFORMITY_BASIC_INDIVIDUAL,
    objects,
    sizeof(objects) / sizeof(objects[0]));
```

## Request PDU

```text
+----------+----------+----------------+-----------+
| Function | MEI type | Read Dev ID code | Object ID |
+----------+----------+----------------+-----------+
|   0x2B   |   0x0E   | 01 / 02 / 03 / 04 | 1 byte |
+----------+----------+----------------+-----------+
```

The request PDU length is exactly four bytes.

Read Device ID codes:

- `0x01`: Basic stream access;
- `0x02`: Regular stream access;
- `0x03`: Extended stream access;
- `0x04`: Specific Object access.

For stream access, Object ID selects the first object. An unknown Object ID
restarts the stream at object `0x00`, as required by the specification. If the
requested description level is above the configured conformity level, the
server responds using its actual lower level.

For Specific Object access, an unknown or unavailable object returns Illegal
Data Address (`0x02`). A stream-only configuration rejects Specific Object
access with Illegal Data Value (`0x03`).

## Response PDU

```text
Function                 1 byte  0x2B
MEI type                 1 byte  0x0E
Read Device ID code      1 byte  echo of request
Conformity level         1 byte  01/02/03 or 81/82/83
More Follows             1 byte  00 or FF
Next Object ID           1 byte
Number of objects        1 byte
Object list              variable
```

Each object is encoded as:

```text
Object ID | Object length | Object value bytes
 1 byte   |    1 byte     | object length bytes
```

Objects are indivisible. The server adds complete objects in ascending ID order
until the next object would exceed the supplied response capacity. It then sets
`More Follows` to `0xFF` and returns that object's ID in `Next Object ID`. A
follow-up request uses that ID as its starting object.

If even the first requested object cannot fit, the core returns Server Device
Failure (`0x04`). Configuration prevents an individual object from exceeding a
maximum Modbus PDU.

Specific Object responses always contain exactly one object and set both
`More Follows` and `Next Object ID` to zero.

## Server exception mapping

- Illegal Data Address (`0x02`): unknown Specific Object;
- Illegal Data Value (`0x03`): wrong request length, wrong MEI type, illegal
  Read Device ID code, or unsupported Specific Object access;
- Server Device Failure (`0x04`): no active object configuration or insufficient
  normal-response capacity.

## RTU master API

The request builder is:

```c
int mbrtum_build_read_device_identification_request(
    uint8_t slave_address,
    uint8_t read_device_id_code,
    uint8_t object_id,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);
```

It accepts unicast slave addresses 1 through 247, validates the access code,
builds the seven-byte RTU request ADU, and appends the low-byte-first CRC.

The request descriptor stores:

```text
function       0x2B
start_address  Read Device ID code
quantity       requested Object ID
value          MEI type 0x0E
expects_response 1
```

`mbrtum_process_response()` validates:

- complete ADU length and CRC;
- slave address and function;
- MEI type and echoed Read Device ID code;
- conformity level;
- More Follows and Next Object ID consistency;
- nonzero object count;
- exact object boundaries and total response length;
- strictly increasing object IDs;
- object category for Basic, Regular, and Extended stream access;
- exact requested object and advertised category for Specific Object access.

The generic response view references the caller-owned ADU. Use:

```c
mbrtum_device_id_response_t view;
mbrtum_device_id_object_t object;

mbrtum_get_device_id_response(&response, &view);
mbrtum_get_device_id_object(&view, 0u, &object);
```

Both helpers are zero-copy. Object values remain valid while the response ADU
remains unchanged.

## Transaction engine

`mbrtum_transaction_start()` accepts FC43/14 requests generated by the builder.
Before transmission it checks the descriptor, exact seven-byte request ADU,
MEI type, access code, object ID, response expectation, and CRC. The existing
asynchronous transmit, timeout, retry, exception, cancellation, and response
accounting behavior is reused unchanged.

## Transport behavior

### Modbus TCP

FC43/14 is handled by the shared PDU engine. The TCP wrapper remains unchanged
and preserves the request transaction identifier, protocol identifier, and unit
identifier.

### Modbus RTU

Unicast requests receive normal or exception responses with a newly generated
CRC. Address-zero broadcast requests are silently ignored and do not affect the
object model because FC43/14 is a read operation.

## Verification

`Tests/host/test_modbus_fc43_device_id.c` covers:

- configuration validation and fixed-capacity state;
- Basic, Regular, Extended, and Specific Object access;
- conformity-level fallback;
- unknown stream-object restart behavior;
- deterministic segmentation and follow-up requests;
- response-capacity failure without partial objects;
- malformed request length, MEI type, and access code;
- Modbus TCP and Modbus RTU framing;
- RTU broadcast suppression;
- maximum 253-byte response PDU and 256-byte RTU ADU;
- master request builder, CRC, parser, and zero-copy object iteration;
- malformed master responses;
- master transaction-engine completion and descriptor consistency.

## Reference

- Modbus Application Protocol Specification V1.1b3, section 6.21:
  <https://www.modbus.org/file/secure/modbusprotocolspecification.pdf>
