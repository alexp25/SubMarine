#include "twi_utils.h"

/*
 * read bytes from chip register
 */

int8_t wire_read_reg_bytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
    wire_begin_transmission(addr);
    wire_write_byte(reg);
    wire_end_transmission();
    wire_request_from(addr, length, 1);

    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = wire_read();
    }

    return 0;
}

/*
 * read bytes from chip register
 */
int8_t wire_read_reg_byte(uint8_t addr, uint8_t regAddr, uint8_t *data)
{
    wire_begin_transmission(addr);
    wire_write_byte(regAddr);
    wire_end_transmission();
    wire_request_from(addr, 1, 1);
    *data = wire_read();
    return 1;
}

/*
 * read bytes from chip register
 */
int8_t wire_read_reg_16(uint8_t addr, uint8_t regAddr, uint16_t *data)
{
    wire_begin_transmission(addr);
    wire_write_byte(regAddr);
    wire_end_transmission();
    wire_request_from(addr, 2, 1);
    *data = wire_read();
    *data = (*data << 8) + wire_read();
    return 1;
}

/*
 * write bytes from chip register
 */
void wire_write_reg_bytes(uint8_t addr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    if (length > 0)
    {
        wire_begin_transmission(addr);
        wire_write_byte(regAddr);
        for (uint8_t i = 0; i < length; i++)
        {
            wire_write_byte(data[i]);
        }
        wire_end_transmission();
    }
}

/*
 * write byte to chip register
 */
void wire_write_reg_byte(uint8_t addr, uint8_t regAddr, uint8_t data)
{
    wire_begin_transmission(addr);
    wire_write_byte(regAddr);
    wire_write_byte(data);
    wire_end_transmission();
}

/*
 * write 2 bytes to chip register
 */
void wire_write_reg_16(uint8_t addr, uint8_t regAddr, uint16_t data)
{
    wire_begin_transmission(addr);
    wire_write_byte(regAddr);
    wire_write_byte((uint8_t)(data >> 8));
    wire_write_byte((uint8_t)(data & 0xFF));
    wire_end_transmission();
}