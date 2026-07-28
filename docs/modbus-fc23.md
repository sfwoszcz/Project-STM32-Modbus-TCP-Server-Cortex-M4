# FC23 Read/Write Multiple Registers

This milestone adds portable support for Modbus function code 23 (`0x17`),
Read/Write Multiple Registers, to the shared protocol core and the complete-frame
RTU master APIs.

FC23 is a normal public Modbus application function. It is available through
both supported server transports:

```text
Modbus TCP ADU -> mbtcp_process_adu() -> mb_process_pdu()
Modbus RTU ADU -> mbrtu_process_adu()  -> mb_process_pdu()
```

The serial-line-only diagnostics dispatcher is not involved.

## Semantics

FC23 combines one holding-register write and one holding-register read in a
single transaction. The write operation is completed before the read operation.
This ordering is observable when the two ranges overlap: the response contains
the newly written values for overlapping registers.

The portable core performs no dynamic allocation and uses only caller-owned
request and response buffers plus the existing fixed-size holding-register map.

## Request PDU

```text
+----------+--------------------+---------------+---------------------+
| Function | Read start address | Read quantity | Write start address |
+----------+--------------------+---------------+---------------------+
|  0x17    |      2 bytes       |    2 bytes    |       2 bytes       |
+----------+--------------------+---------------+---------------------+

+----------------+------------------+------------------------------+
| Write quantity | Write byte count | Write register values        |
+----------------+------------------+------------------------------+
|    2 bytes     |      1 byte      | 2 bytes per write register   |
+----------------+------------------+------------------------------+
```

All addresses, quantities, and register values use big-endian Modbus wire
order.

Limits:

- read quantity: 1 through 125 registers
- write quantity: 1 through 121 registers
- write byte count: exactly `write quantity * 2`
- request PDU length: exactly `10 + write byte count`
- maximum request PDU length: 252 bytes

Both address ranges are validated independently. A range is valid only when its
last register remains inside the configured holding-register map.

## Response PDU

```text
+----------+-----------------+----------------------------+
| Function | Read byte count | Read register values       |
+----------+-----------------+----------------------------+
|  0x17    |     1 byte      | 2 bytes per read register  |
+----------+-----------------+----------------------------+
```

The response byte count is exactly `read quantity * 2`. The maximum normal
response PDU length is 252 bytes.

## Server processing

`mb_process_pdu()` validates the complete request before changing any holding
register:

1. exact minimum and final request length;
2. read quantity from 1 through 125;
3. write quantity from 1 through 121;
4. exact write byte count;
5. both configured holding-register address ranges;
6. response capacity for the complete normal response.

Only after every check succeeds does the core apply the writes. It then reads
the requested range and builds the response. Therefore malformed requests,
invalid addresses, and insufficient response capacity do not partially apply a
write.

Exception mapping follows the existing shared PDU rules:

- Illegal Data Address (`0x02`) for an invalid read or write map range;
- Illegal Data Value (`0x03`) for malformed lengths, quantities, or byte count;
- Server Device Failure (`0x04`) when the supplied response buffer cannot hold
  the normal response.

## Transport behavior

### Modbus TCP

FC23 is dispatched through the shared PDU engine. `mbtcp_process_adu()` performs
its existing MBAP validation and returns the FC23 response using the same
transaction identifier, protocol identifier, and unit identifier.

### Modbus RTU slave

Unicast FC23 requests are processed by the ordinary RTU path and receive a CRC
protected response.

Address-zero broadcast FC23 requests are ignored and do not modify the holding
register map. FC23 requires a read response, so it is not included in the RTU
broadcast-write allowlist.

## RTU master request builder

The public builder is:

```c
int mbrtum_build_read_write_multiple_registers_request(
    uint8_t slave_address,
    uint16_t read_start_address,
    uint16_t read_quantity,
    uint16_t write_start_address,
    uint16_t write_quantity,
    const uint16_t *write_values,
    mbrtum_request_t *request,
    uint8_t *request_adu,
    size_t request_adu_capacity,
    size_t *request_adu_length);
```

The builder:

- accepts only unicast slave addresses 1 through 247;
- validates both 16-bit address ranges;
- enforces the 125-register read and 121-register write limits;
- encodes host-endian `uint16_t` values in big-endian wire order;
- checks output capacity before writing the ADU;
- appends the low-byte-first RTU CRC;
- produces a maximum-size 255-byte RTU request ADU.

The generated request descriptor uses:

```text
start_address       FC23 read starting address
quantity            FC23 read quantity
write_start_address FC23 write starting address
write_quantity      FC23 write quantity
value               zero
expects_response    one
```

FC23 broadcast builders are rejected because a valid transaction requires a
response.

## Master response validation

`mbrtum_process_response()` validates:

- complete RTU length and CRC;
- slave address;
- normal, exception, or mismatched function code;
- exact response length for the requested read quantity;
- exact response byte count.

The normal response view references the register bytes directly in the
caller-owned response ADU. `mbrtum_get_register()` supports FC23 and decodes one
zero-based register index without copying the response payload.

## Transaction engine

`mbrtum_transaction_start()` validates the FC23 descriptor against the complete
request ADU before transmission. It checks both address/quantity pairs, the
write byte count, exact request length, response expectation, and CRC. The
existing timeout, retry, cancellation, exception, and response-accounting state
machine is reused unchanged.

## Verification

`Tests/host/test_modbus_fc23.c` covers:

- shared PDU write-before-read behavior with overlapping ranges;
- strict quantity, byte-count, length, address, and capacity checks;
- no write on malformed input or insufficient response capacity;
- maximum 125-register read and 121-register write request;
- Modbus TCP request and response framing;
- Modbus RTU unicast framing and CRC;
- RTU broadcast suppression without register modification;
- RTU master builder fields, limits, capacity, and CRC;
- maximum 255-byte RTU master request;
- exact master response validation and zero-copy register decoding;
- RTU master transaction-engine completion and descriptor/ADU consistency.

Run the complete suite with:

```sh
make clean
make test
```

Or with CMake and CTest:

```sh
cmake -S . -B build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-cmake
ctest --test-dir build-cmake --output-on-failure
```

## Reference

- Modbus Application Protocol Specification V1.1b3, section 6.17:
  <https://www.modbus.org/file/secure/modbusprotocolspecification.pdf>
