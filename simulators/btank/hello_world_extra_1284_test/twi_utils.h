
#ifndef _TWI_UTILS_H_
#define _TWI_UTILS_H_

#include <avr/io.h>
#include <util/delay.h>
#include "twi_arduino.h"

#ifdef __cplusplus
extern "C"
{
#endif

    int8_t wire_read_reg_bytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data);
    int8_t wire_read_reg_byte(uint8_t addr, uint8_t regAddr, uint8_t *data);
    int8_t wire_read_reg_16(uint8_t addr, uint8_t regAddr, uint16_t *data);
    void wire_write_reg_bytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data);
    void wire_write_reg_byte(uint8_t addr, uint8_t regAddr, uint8_t data);
    void wire_write_reg_16(uint8_t addr, uint8_t regAddr, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif