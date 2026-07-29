# Modbus FC24 Read FIFO Queue

## Scope

FC24 (`0x18`) reads a non-destructive snapshot of one configured FIFO queue.
The shared transport-independent PDU implementation is available through both
Modbus TCP and unicast Modbus RTU. Broadcast RTU reads are ignored and do not
produce a response.

The implementation uses no heap allocation. Applications provide fixed
register blocks that remain valid while configured.

## Wire format

The request PDU is exactly three bytes:

| Field | Size |
|---|---:|
| Function code `0x18` | 1 byte |
| FIFO Pointer Address | 2 bytes |

A normal response is:

| Field | Size |
|---|---:|
| Function code `0x18` | 1 byte |
| Byte Count | 2 bytes |
| FIFO Count | 2 bytes |
| FIFO Value Registers | `FIFO Count × 2` bytes |

The Byte Count includes the two FIFO Count bytes and all returned value bytes:

```text
Byte Count = 2 + (FIFO Count × 2)
```

FIFO Count may be zero through 31. The largest response contains a Byte Count
of 64 and a 67-byte PDU.

## Application storage model

Include `modbus_fifo.h` and configure one or more sorted descriptors:

```c
#include "modbus_fifo.h"

static uint16_t process_fifo[] = {
    2u,       /* FIFO Count */
    0x01B8u,  /* first queued value */
    0x1284u   /* second queued value */
};

static const mb_fifo_queue_t queues[] = {
    {
        .pointer_address = 0x04DEu,
        .registers = process_fifo,
        .register_count = sizeof(process_fifo) / sizeof(process_fifo[0])
    }
};

void app_modbus_init(void)
{
    mb_init();
    (void)mb_fifo_configure(queues,
                            sizeof(queues) / sizeof(queues[0]));
}
```

`registers[0]` is the current FIFO Count. `registers[1..count]` are the queued
values in the order returned. `register_count` includes the count register and
must be between 1 and 32.

The core copies only the descriptors. The register data remains
application-owned, so applications may update a queue without reconfiguring
the map. FC24 reads do not modify, pop, or clear any value.

When `WITH_RTOS` is enabled, update the application-owned block while holding
the same `mb_lock()`/`mb_unlock()` used by the core so each response is a
consistent snapshot.

## Configuration API

```c
int mb_fifo_configure(const mb_fifo_queue_t *queues, size_t queue_count);
void mb_fifo_clear(void);
int mb_fifo_is_configured(void);
```

The fixed descriptor map accepts up to `MB_FIFO_MAX_QUEUES` queues. Descriptors
must be sorted by strictly increasing FIFO Pointer Address. A failed
configuration leaves the previous map unchanged. `mb_init()` clears the FIFO
map.

## Exception behavior

| Condition | Exception |
|---|---:|
| Request PDU is not exactly 3 bytes | Illegal Data Value `0x03` |
| FIFO Pointer Address is not configured | Illegal Data Address `0x02` |
| FIFO Count exceeds 31 | Illegal Data Value `0x03` |
| Configured storage does not contain `count + 1` registers | Server Device Failure `0x04` |
| Response buffer cannot hold the complete response | Server Device Failure `0x04` |

## RTU master API

Build a request with:

```c
mbrtum_request_t request;
uint8_t adu[MODBUS_RTU_ADU_MAX_SIZE];
size_t adu_length;

int result = mbrtum_build_read_fifo_queue_request(
    1u,
    0x04DEu,
    &request,
    adu,
    sizeof(adu),
    &adu_length);
```

The builder accepts unicast slave addresses 1 through 247. It generates the
complete six-byte RTU request ADU, including CRC.

After `mbrtum_process_response()` validates a normal reply, decode it without
copying:

```c
mbrtum_response_t response;
mbrtum_fifo_response_t fifo;
uint16_t value;

(void)mbrtum_get_fifo_response(&response, &fifo);
(void)mbrtum_get_fifo_register(&fifo, 0u, &value);
```

The response validator checks CRC, slave address, function, the two-byte Byte
Count, FIFO Count, count limit, exact `2 + 2N` relationship, and exact ADU
length.

## Transaction engine

`mbrtum_transaction_start()` accepts FC24 requests produced by the builder. It
validates the exact six-byte ADU, FIFO Pointer Address, unused descriptor
fields, response expectation, and CRC before calling the transport. Normal,
exception, unrelated, malformed, timeout, and retry handling reuse the common
transaction engine.

## Verification

`Tests/host/test_modbus_fc24.c` covers:

- descriptor configuration and failed-configuration preservation;
- the specification example at pointer `0x04DE`;
- empty, one-value, and 31-value queues;
- non-destructive repeated reads;
- malformed request lengths;
- unknown pointer addresses;
- FIFO Count greater than 31;
- inconsistent application storage and response-capacity failures;
- Modbus TCP and RTU unicast responses;
- ignored RTU broadcasts;
- master request construction and CRC;
- response Byte Count/FIFO Count consistency;
- zero-copy decoding and index checks;
- exception, CRC, address, function, and malformed-response handling;
- transaction-engine request and response handling.
