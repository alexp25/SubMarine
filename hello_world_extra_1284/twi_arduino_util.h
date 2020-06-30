/*
  twi.h - TWI/I2C library for Wiring & Arduino
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
*/

#ifndef _AVR_TWI_ARDUINO_UTILS_H_
#define _AVR_TWI_ARDUINO_UTILS_H_

// #include <math.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#ifdef __cplusplus
extern "C"
{
#endif

    //#define ATMEGA8

#define DDR_TWI DDRC
#define PORT_TWI PORTC
#define PIN_SDA PC1
#define PIN_SCL PC0

#ifndef TWI_FREQ
#define TWI_FREQ 100000L
// #define TWI_FREQ 400000L
#endif

#ifndef TWI_BUFFER_LENGTH
#define TWI_BUFFER_LENGTH 32
#endif

#define TWI_READY 0
#define TWI_MRX 1
#define TWI_MTX 2
#define TWI_SRX 3
#define TWI_STX 4

    extern volatile uint8_t twi_state;
    extern volatile uint8_t twi_slarw;
    extern volatile uint8_t twi_sendStop;   // should the transaction end with a stop
    extern volatile uint8_t twi_inRepStart; // in the middle of a repeated start

    extern void (*twi_onSlaveTransmit)(void);
    extern void (*twi_onSlaveReceive)(uint8_t *, int);

    extern uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
    extern volatile uint8_t twi_masterBufferIndex;
    extern volatile uint8_t twi_masterBufferLength;

    extern uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
    extern volatile uint8_t twi_txBufferIndex;
    extern volatile uint8_t twi_txBufferLength;

    extern uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
    extern volatile uint8_t twi_rxBufferIndex;

    extern volatile uint8_t twi_error;

    void twi_init(void);
    void twi_disable(void);
    void twi_setAddress(uint8_t);
    void twi_setFrequency(uint32_t);
    uint8_t twi_readFrom(uint8_t, uint8_t *, uint8_t, uint8_t);
    uint8_t twi_writeTo(uint8_t, uint8_t *, uint8_t, uint8_t, uint8_t);
    uint8_t twi_transmit(const uint8_t *, uint8_t);
    void twi_attachSlaveRxEvent(void (*)(uint8_t *, int));
    void twi_attachSlaveTxEvent(void (*)(void));
    void twi_reply(uint8_t);
    void twi_stop(void);
    void twi_releaseBus(void);

#ifdef __cplusplus
}
#endif

#endif
