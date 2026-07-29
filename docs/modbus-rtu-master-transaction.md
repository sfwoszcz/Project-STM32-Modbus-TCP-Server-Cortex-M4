# Portable Modbus RTU master transaction engine

This layer coordinates one complete Modbus RTU master transaction without
calling an operating system, UART driver, STM32 HAL, or RS-485 GPIO.

It owns an immutable copy of the request descriptor and request ADU, starts
transmission through a caller callback, waits for an explicit transmit-complete
or transmit-error event, applies a wrap-safe response deadline, and validates
complete response ADUs through `mbrtum_process_response()`.

## States

- `IDLE`: initialized and not yet used.
- `TRANSMITTING`: the transport accepted the ADU and must report completion.
- `WAITING_RESPONSE`: transmission completed and the response deadline is active.
- `RETRY_DELAY`: a retry is eligible at or after the configured deadline.
- `COMPLETE`: normal response, Modbus exception, or no-response request completed.
- `FAILED`: timeout, response error, or transport failure exhausted the policy.
- `CANCELLED`: the application cancelled an active transaction.

A new transaction may start from any non-active state. Only one transaction can
be active in a context. The context must be initialized before any other API is
used, must not be copied after initialization, and must not be reinitialized
while a transport operation is active.

## Timing

All times are caller-defined unsigned 32-bit ticks. Durations must not exceed
`INT32_MAX`; deadline comparisons remain valid across one 32-bit counter wrap.
The engine never sleeps or reads a hardware clock. The application calls
`mbrtum_transaction_poll()` with the current monotonic tick.

`max_retries` counts additional attempts. Zero means one total transmission;
two means up to three total transmissions.

A retry, including a retry with a configured delay of zero, is started only by a
later `mbrtum_transaction_poll()` call. This prevents recursive transport calls
and bounds each public API invocation to at most one transmission attempt.

## Transport contract

The transmit callback must either accept the complete ADU and return
`MBRTUM_TXN_TRANSMIT_ACCEPTED`, or return another value for an immediate
transport failure. Acceptance does not mean the bytes have left the peripheral.
The platform later calls `mbrtum_transaction_on_tx_complete()` or
`mbrtum_transaction_on_tx_error()`.

A request whose descriptor says that no response is expected completes after
transmit completion and never starts a response deadline. Broadcast writes,
including FC21 and FC22, use this behavior; the transaction layer does not
hard-code a specific function code.

## FC11 support

The transaction engine accepts FC11 requests produced by
`mbrtum_build_report_server_id_request()`. Before transmission it validates the
unicast address, function-only four-byte ADU, device-specific expected Server
ID length, zero-valued unused descriptor fields, response expectation, and
CRC. Normal, exception, malformed, unrelated, timeout, and retry handling reuse
the existing state machine.

## FC20 support

The transaction engine accepts FC20 requests produced by
`mbrtum_build_read_file_record_request()`. Before transmission it checks the
request byte count, subrequest count, every reference/file/record field,
combined expected response size, exact ADU length, response expectation, and
CRC against the immutable request descriptor. Response validation uses the
transaction-owned request ADU to match each returned subresponse length.

## FC21 support

The transaction engine accepts FC21 requests produced by
`mbrtum_build_write_file_record_request()`. Before transmission it checks the
request-data byte count, variable subrequest boundaries, reference/file/record
fields, record-data extents, subrequest count, response expectation, exact ADU
length, and CRC against the immutable request descriptor.

Unicast responses are matched byte-for-byte against the transaction-owned
request ADU. Broadcast FC21 requests complete after transmit completion and do
not start a response deadline.

## FC22 support

The transaction engine accepts FC22 requests produced by
`mbrtum_build_mask_write_register_request()`. Before transmission it validates
the exact ten-byte ADU, register address, AND mask, OR mask, response
expectation, and CRC against the immutable request descriptor. Unicast replies
reuse the exact acknowledgement validator. Broadcast requests complete after
transmit completion without starting a response deadline.

## FC24 support

The transaction engine accepts FC24 requests produced by
`mbrtum_build_read_fifo_queue_request()`. Before transmission it validates the
exact six-byte ADU, unicast address, FIFO Pointer Address, zero-valued unused
descriptor fields, response expectation, and CRC against the immutable request
descriptor. Normal, exception, malformed, unrelated, timeout, and retry
handling reuse the common state machine.

## FC23 support

The transaction engine accepts FC23 requests produced by
`mbrtum_build_read_write_multiple_registers_request()`. Before transmission it
checks the read and write ranges, both quantities, exact request length, write
byte count, response expectation, function, address, and CRC against the
request descriptor. Response timing, retries, exceptions, and decoding reuse
the existing state machine.

## FC43/14 support

The transaction engine accepts requests produced by
`mbrtum_build_read_device_identification_request()`. Before transmission it
checks the exact seven-byte ADU, unicast address, function, MEI type, access
code, object ID, response expectation, and CRC against the immutable request
descriptor. Normal, exception, malformed, unrelated, timeout, and retry behavior
reuse the existing state machine.

## Request validation

Before transmission, the engine checks that:

- the ADU length fits the fixed transaction-owned buffer;
- the slave address and function byte match the request descriptor;
- the slave address is within the serial unicast/broadcast range;
- address zero is never marked as expecting a response;
- the complete request ADU has a valid low-byte-first Modbus CRC.

An invalid or inconsistent request is rejected without calling the transport.

## Response policy

Complete frames are submitted with `mbrtum_transaction_on_response()`.

- A valid normal response completes successfully.
- A valid Modbus exception completes as `MODBUS_EXCEPTION` and is not retried.
- Address or function mismatches are counted as unrelated and ignored.
- CRC, length, acknowledgement, byte-count, and other malformed responses are
  counted and ignored until the deadline.
- A frame submitted at or after the response deadline is counted as late and is
  not accepted as a successful response.
- If the final deadline expires after at least one malformed candidate frame in
  that attempt, the final result is `RESPONSE_ERROR`; otherwise it is `TIMEOUT`.

The response ADU is copied into transaction-owned storage before validation, so
the validated response view remains stable after the caller reuses its receive
buffer. Response storage and per-attempt response status are cleared before
each retry.

## Memory and resilience

- no dynamic allocation;
- fixed maximum Modbus RTU ADU storage;
- checked request and response lengths;
- request descriptor/ADU consistency and CRC validation;
- saturating diagnostic counters;
- immutable active request copy;
- explicit initialization and state checks for asynchronous events;
- poll-driven, non-recursive retries;
- late-response rejection;
- no hardware-specific includes or callbacks beyond the abstract transmitter.

RS-485 DE/RE handling belongs in a future transport adapter, not this engine.
