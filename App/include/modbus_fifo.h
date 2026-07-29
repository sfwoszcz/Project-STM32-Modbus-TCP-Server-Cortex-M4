#ifndef MODBUS_FIFO_H
#define MODBUS_FIFO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MB_FIFO_FUNCTION_CODE 0x18u
#define MB_FIFO_MAX_QUEUES 8u
#define MB_FIFO_MAX_VALUES 31u
#define MB_FIFO_MAX_REGISTERS (1u + MB_FIFO_MAX_VALUES)
#define MB_FIFO_MAX_BYTE_COUNT (2u + (MB_FIFO_MAX_VALUES * 2u))
#define MB_FIFO_MAX_RESPONSE_PDU_SIZE (3u + MB_FIFO_MAX_BYTE_COUNT)

#define MB_FIFO_OK 0
#define MB_FIFO_ERROR_ARGUMENT (-1)
#define MB_FIFO_ERROR_CAPACITY (-2)
#define MB_FIFO_ERROR_QUEUES (-3)

/**
 * Application-owned register block for one FC24 FIFO queue.
 *
 * pointer_address is the 16-bit FIFO Pointer Address used by the request.
 * registers[0] is the current FIFO Count and registers[1..count] are the
 * queued values in read order. register_count is the total number of valid
 * uint16_t entries in registers, including the count register, and must be
 * between 1 and 32.
 *
 * The portable core copies only these descriptors. The register blocks remain
 * application-owned and must stay valid while configured. Reads are snapshots
 * and do not clear or consume the queue. When WITH_RTOS is enabled, update the
 * register block while holding the same mb_lock()/mb_unlock() used by the core.
 */
typedef struct {
    uint16_t pointer_address;
    const uint16_t *registers;
    size_t register_count;
} mb_fifo_queue_t;

/**
 * Configure the fixed-capacity FC24 FIFO map.
 *
 * queues must be sorted by strictly increasing pointer_address. Failed
 * configuration leaves the existing map unchanged.
 */
int mb_fifo_configure(const mb_fifo_queue_t *queues, size_t queue_count);

/** Clear all configured FIFO descriptors. */
void mb_fifo_clear(void);

/** Return non-zero when at least one FIFO is configured. */
int mb_fifo_is_configured(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_FIFO_H */
