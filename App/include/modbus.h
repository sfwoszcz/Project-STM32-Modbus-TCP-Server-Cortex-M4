#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>
#include "platform_port.h"

#ifdef __cplusplus
extern "C" {
#endif

void mb_init(void);

uint8_t mb_get_coil(uint16_t address);
void mb_set_coil(uint16_t address, uint8_t value);
uint8_t mb_get_dinput(uint16_t address);
void mb_set_dinput(uint16_t address, uint8_t value);

uint16_t mb_get_hreg(uint16_t address);
void mb_set_hreg(uint16_t address, uint16_t value);
/* Atomically apply the Modbus FC22 mask formula and return the new value. */
uint16_t mb_mask_write_hreg(uint16_t address,
                            uint16_t and_mask,
                            uint16_t or_mask);
uint16_t mb_get_ireg(uint16_t address);
void mb_set_ireg(uint16_t address, uint16_t value);

/* Override these weak hooks to connect writes to application hardware. */
void mb_on_write_hreg(uint16_t address, uint16_t value);
void mb_on_write_coil(uint16_t address, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_H */
