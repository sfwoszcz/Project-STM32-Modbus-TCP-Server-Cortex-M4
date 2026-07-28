#ifndef MODBUS_FILE_RECORD_H
#define MODBUS_FILE_RECORD_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MB_FILE_RECORD_READ_FUNCTION_CODE 0x14u
#define MB_FILE_RECORD_WRITE_FUNCTION_CODE 0x15u
#define MB_FILE_RECORD_REFERENCE_TYPE 0x06u
#define MB_FILE_RECORD_MAX_FILES 8u
#define MB_FILE_RECORD_MAX_RECORDS_PER_FILE 10000u
#define MB_FILE_RECORD_MAX_SUBREQUESTS 35u
#define MB_FILE_RECORD_REQUEST_DATA_MAX 245u
#define MB_FILE_RECORD_RESPONSE_DATA_MAX 245u
#define MB_FILE_RECORD_WRITE_MAX_SUBREQUESTS 27u
#define MB_FILE_RECORD_WRITE_REQUEST_DATA_MAX 251u
#define MB_FILE_RECORD_WRITE_RESPONSE_DATA_MAX 251u

#define MB_FILE_RECORD_OK 0
#define MB_FILE_RECORD_ERROR_ARGUMENT (-1)
#define MB_FILE_RECORD_ERROR_CAPACITY (-2)
#define MB_FILE_RECORD_ERROR_FILES (-3)

/**
 * Application-owned storage for one Modbus file.
 *
 * file_number must be 1-65535. record_count must be 1-10000. records must
 * point to record_count 16-bit values and remain valid while the file map is
 * configured. The portable core copies only these descriptors; it does not
 * copy the record data or allocate memory.
 *
 * The pointer is writable because FC21 Write File Record updates the same
 * configured application-owned storage used by FC20 reads.
 */
typedef struct {
    uint16_t file_number;
    uint16_t *records;
    size_t record_count;
} mb_file_record_file_t;

/**
 * Configure the fixed-capacity file-record map.
 *
 * files must be sorted by strictly increasing file_number. Failed
 * configuration leaves the existing map unchanged.
 */
int mb_file_record_configure(const mb_file_record_file_t *files,
                             size_t file_count);

/** Clear all configured file-record descriptors. */
void mb_file_record_clear(void);

/** Return non-zero when at least one file is configured. */
int mb_file_record_is_configured(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_FILE_RECORD_H */
