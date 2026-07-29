# Modbus FC22 Mask Write Register

FC22 (`0x16`) modifies one holding register by combining its current value with
an AND mask and an OR mask. The function is supported by the shared PDU engine,
so it is available through Modbus TCP and Modbus RTU. RTU broadcast requests
are accepted and applied without a response.

## Request and response PDU

The request PDU is exactly seven bytes:

```text
function  address high  address low  AND high  AND low  OR high  OR low
  0x16        1 byte       1 byte      1 byte    1 byte   1 byte   1 byte
```

A successful unicast response echoes the complete request PDU exactly.

The register update is:

```text
result = (current AND and_mask) OR (or_mask AND (NOT and_mask))
```

This means:

- a `1` bit in the AND mask preserves the corresponding current bit;
- a `0` bit in the AND mask allows the corresponding OR-mask bit to replace it;
- an AND mask of `0xFFFF` leaves the register unchanged;
- an AND mask of `0x0000` replaces the register with the OR mask.

## Slave behavior

The shared PDU engine validates, in order:

1. exact seven-byte request length;
2. holding-register address range;
3. response capacity for the seven-byte echo.

Only after all checks succeed does the implementation update the register. The
read-modify-write operation is performed by `mb_mask_write_hreg()` while the
portable register-map lock is held, then the existing `mb_on_write_hreg()` hook
is called once with the final value.

Errors use standard Modbus exceptions:

- Illegal Data Address (`0x02`) for an unavailable holding-register address;
- Illegal Data Value (`0x03`) for an incorrect request length;
- Server Device Failure (`0x04`) when the response buffer cannot hold the echo.

## Modbus RTU behavior

A unicast RTU request returns the exact FC22 PDU echo with a newly generated
low-byte-first CRC.

Address-zero RTU broadcast requests are validated and applied, but no response
is generated. Invalid broadcast requests do not modify the register map.

## RTU master API

Build a request with:

```c
mbrtum_request_t request;
uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
size_t request_adu_length = 0u;

int result = mbrtum_build_mask_write_register_request(
    1u,
    10u,
    0x00FFu,
    0xFF00u,
    &request,
    request_adu,
    sizeof(request_adu),
    &request_adu_length);
```

The compact request descriptor stores:

- `start_address`: register address;
- `quantity`: AND mask;
- `value`: OR mask;
- `expects_response`: zero for broadcast, one for unicast.

`mbrtum_process_response()` accepts a normal response only when the address,
AND mask, and OR mask exactly match the request descriptor. Broadcast request
descriptors return `MBRTUM_ERROR_RESPONSE_NOT_EXPECTED` if response validation
is attempted.

The transaction engine validates the descriptor against the complete ten-byte
request ADU, including address, both masks, response expectation, and CRC.
Unicast requests enter the normal transmit/wait/validate path. Broadcast
requests complete after transmit completion without starting a response
deadline.

## Limits and memory

- one holding register per request;
- fixed seven-byte PDU;
- fixed ten-byte RTU ADU;
- no heap allocation;
- no retained request or response pointers;
- existing register map and write hook are reused.

## Host verification

`Tests/host/test_modbus_fc22.c` covers:

- mask formula and edge masks;
- exactly one write-hook notification per valid request;
- malformed length, invalid address, and response-capacity handling;
- no modification after any validation failure;
- Modbus TCP and RTU unicast processing;
- RTU broadcast application with response suppression;
- master unicast and broadcast request generation;
- exact response echo, exceptions, CRC, length, and acknowledgement mismatch;
- transaction-engine request consistency and unicast/broadcast completion.
