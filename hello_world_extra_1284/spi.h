#ifndef _SPI_H_
#define _SPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>


/* SPI config */
#define SPI_PORT	PORTB
#define SPI_DDR 	DDRB
#define SPI_MISO	PB6
#define SPI_MOSI	PB5
#define SPI_SCK 	PB7
#define SPI_SS      PB4

void SPI_init(uint8_t msbfirst, uint8_t mode, uint8_t clkdiv);
uint8_t SPI_exchange(uint8_t data);
void SPI_transmit(uint8_t *data, uint8_t length);
void SPI_request(uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif

