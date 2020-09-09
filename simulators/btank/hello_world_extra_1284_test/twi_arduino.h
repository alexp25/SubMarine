/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

#ifndef _AVR_TWI_ARDUINO_H_
#define _AVR_TWI_ARDUINO_H_

#include <string.h>
#include <inttypes.h>
#include "twi_arduino_util.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BUFFER_LENGTH 32

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

extern uint8_t rxBuffer[];
extern uint8_t rxBufferIndex;
extern uint8_t rxBufferLength;

extern uint8_t txAddress;
extern uint8_t txBuffer[];
extern uint8_t txBufferIndex;
extern uint8_t txBufferLength;

extern uint8_t transmitting;

extern void (*user_onRequest)(void);
extern void (*user_onReceive)(int);
extern void wire_onrequest_service(void);
extern void wire_onreceive_service(uint8_t *, int);

void wire_begin();
void wire_begin_addr(uint8_t addr);
void wire_end();
void wire_set_clock(uint32_t clock);
void wire_begin_transmission(uint8_t address);
uint8_t wire_end_transmission(void);
uint8_t wire_end_transmission_options(uint8_t sendStop);
uint8_t wire_request_from(uint8_t address, uint8_t quantity, uint8_t sendStop);
uint8_t wire_request_from_options(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop);

size_t wire_write_byte(uint8_t data);
size_t wire_write(uint8_t * data, size_t quantity);
int wire_available(void);
int wire_read(void);
int wire_peek(void);
void wire_flush(void);

void wire_onerceive(void (*)(int));
void wire_onrequest(void (*)(void));

// inline size_t wire_write(unsigned long n) { return write((uint8_t)n); }
// inline size_t wire_write(long n) { return write((uint8_t)n); }
// inline size_t wire_write(unsigned int n) { return write((uint8_t)n); }
// inline size_t wire_write(int n) { return write((uint8_t)n); }

#ifdef __cplusplus
}
#endif

#endif
