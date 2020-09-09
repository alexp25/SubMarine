/*
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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
  Modified 2017 by Chuck Todd (ctodd@cableone.net) to correct Unconfigured Slave Mode reboot
*/

#include "twi_arduino.h"

// Initialize Class Variables //////////////////////////////////////////////////

uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;
void (*user_onRequest)(void);
void (*user_onReceive)(int);

// Public Methods //////////////////////////////////////////////////////////////

void wire_begin(void)
{
    rxBufferIndex = 0;
    rxBufferLength = 0;

    txBufferIndex = 0;
    txBufferLength = 0;

    twi_init();
    twi_attachSlaveTxEvent(wire_onrequest_service); // default callback must exist
    twi_attachSlaveRxEvent(wire_onreceive_service); // default callback must exist
}

void wire_begin_addr(uint8_t address)
{
    wire_begin();
    twi_setAddress(address);
}

void wire_end(void)
{
    twi_disable();
}

void wire_set_clock(uint32_t clock)
{
    twi_setFrequency(clock);
}

uint8_t wire_request_from_options(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop)
{
    if (isize > 0)
    {
        // send internal address; this mode allows sending a repeated start to access
        // some devices' internal registers. This function is executed by the hardware
        // TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)

        wire_begin_transmission(address);

        // the maximum size of internal address is 3 bytes
        if (isize > 3)
        {
            isize = 3;
        }

        // write internal register address - most significant byte first
        while (isize-- > 0)
        {
            wire_write_byte((uint8_t)(iaddress >> (isize * 8)));
        }
        wire_end_transmission_options(0);
    }

    // clamp to buffer length
    if (quantity > BUFFER_LENGTH)
    {
        quantity = BUFFER_LENGTH;
    }
    // perform blocking read into buffer
    uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
    // set rx buffer iterator vars
    rxBufferIndex = 0;
    rxBufferLength = read;

    return read;
}

uint8_t wire_request_from(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
    // sendStop default 1
    return wire_request_from_options((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)sendStop);
}

void wire_begin_transmission(uint8_t address)
{
    // indicate that we are transmitting
    transmitting = 1;
    // set address of targeted slave
    txAddress = address;
    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling wire_end_transmission_options(0) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to wire_end_transmission_options(1) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t wire_end_transmission_options(uint8_t sendStop)
{
    // transmit buffer (blocking)
    uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, sendStop);
    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
    // indicate that we are done transmitting
    transmitting = 0;
    return ret;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t wire_end_transmission(void)
{
    return wire_end_transmission_options(1);
}

// must be called in:
// slave tx event callback
// or after wire_begin_transmission(address)
size_t wire_write_byte(uint8_t data)
{
    if (transmitting)
    {
        // in master transmitter mode
        // don't bother if buffer is full
        if (txBufferLength >= BUFFER_LENGTH)
        {
            // setWriteError();
            return 0;
        }
        // put byte in tx buffer
        txBuffer[txBufferIndex] = data;
        ++txBufferIndex;
        // update amount in buffer
        txBufferLength = txBufferIndex;
    }
    else
    {
        // in slave send mode
        // reply to master
        twi_transmit(&data, 1);
    }
    return 1;
}

// must be called in:
// slave tx event callback
// or after wire_begin_transmission(address)
size_t wire_write(uint8_t *data, size_t quantity)
{
    if (transmitting)
    {
        // in master transmitter mode
        for (size_t i = 0; i < quantity; ++i)
        {
            wire_write_byte(data[i]);
        }
    }
    else
    {
        // in slave send mode
        // reply to master
        twi_transmit(data, quantity);
    }
    return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int wire_available(void)
{
    return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int wire_read(void)
{
    int value = -1;

    // get each successive byte on each call
    if (rxBufferIndex < rxBufferLength)
    {
        value = rxBuffer[rxBufferIndex];
        ++rxBufferIndex;
    }

    return value;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int wire_peek(void)
{
    int value = -1;

    if (rxBufferIndex < rxBufferLength)
    {
        value = rxBuffer[rxBufferIndex];
    }

    return value;
}

void wire_flush(void)
{
    // XXX: to be implemented.
}

// behind the scenes function that is called when data is received
void wire_onreceive_service(uint8_t *inBytes, int numBytes)
{
    // don't bother if user hasn't registered a callback
    if (!user_onReceive)
    {
        return;
    }
    // don't bother if rx buffer is in use by a master requestFrom() op
    // i know this drops data, but it allows for slight stupidity
    // meaning, they may not have read all the master requestFrom() data yet
    if (rxBufferIndex < rxBufferLength)
    {
        return;
    }
    // copy twi rx buffer into local read buffer
    // this enables new reads to happen in parallel
    for (uint8_t i = 0; i < numBytes; ++i)
    {
        rxBuffer[i] = inBytes[i];
    }
    // set rx iterator vars
    rxBufferIndex = 0;
    rxBufferLength = numBytes;
    // alert user program
    user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void wire_onrequest_service(void)
{
    // don't bother if user hasn't registered a callback
    if (!user_onRequest)
    {
        return;
    }
    // reset tx buffer iterator vars
    // !!! this will kill any pending pre-master sendTo() activity
    txBufferIndex = 0;
    txBufferLength = 0;
    // alert user program
    user_onRequest();
}

// sets function called on slave write
void wire_onreceive(void (*function)(int))
{
    user_onReceive = function;
}

// sets function called on slave read
void wire_onrequest(void (*function)(void))
{
    user_onRequest = function;
}

