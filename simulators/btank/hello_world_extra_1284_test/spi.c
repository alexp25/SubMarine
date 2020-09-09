

#include "spi.h"

// SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3); // 2 MHz, mode 3

void SPI_init(uint8_t msbfirst, uint8_t mode, uint8_t clkdiv)
{
    // set SPI direction: MOSI and SCK as output, MISO as input
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
    SPI_DDR &= ~(1 << SPI_MISO);

    // initialize SPI
    SPCR = (1 << SPE) | // enable
           (1 << MSTR); // master mode

    if (msbfirst)
    {
        SPCR |= (1 << DORD);
    }

    switch (mode)
    {
    case 0:
        SPCR |= (0 << CPOL) | (0 << CPHA);
        break;
    case 1:
        SPCR |= (0 << CPOL) | (1 << CPHA);
        break;
    case 2:
        SPCR |= (1 << CPOL) | (0 << CPHA);
        break;
    case 3:
        SPCR |= (1 << CPOL) | (1 << CPHA);
        break;
    }

    switch (clkdiv)
    {
    // 1x
    case 4:
        SPCR |= (0 << SPR1) | (1 << SPR0);
        SPCR |= (0 << SPR1) | (0 << SPR0);
        break;
    case 16:
        SPCR |= (0 << SPR1) | (1 << SPR0);
        break;
    case 64:
        SPCR |= (1 << SPR1) | (0 << SPR0);
        break;
    case 128:
        SPCR |= (1 << SPR1) | (1 << SPR0);
        break;
    // 2x
    case 2:
        SPSR |= (1 << SPI2X); // f_osc x 2
        SPCR |= (0 << SPR1) | (0 << SPR0);
        break;
    case 8:
        SPSR |= (1 << SPI2X); // f_osc x 2
        SPCR |= (0 << SPR1) | (1 << SPR0);
        break;
    case 32:
        SPSR |= (1 << SPI2X); // f_osc x 2
        SPCR |= (1 << SPR1) | (0 << SPR0);
        break;
    }
}

uint8_t SPI_exchange(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)))
        ;
    return SPDR;
}

void SPI_transmit(uint8_t *data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        SPI_exchange(data[i]);
    }
}

void SPI_request(uint8_t *data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = SPI_exchange(0xff);
    }
}
