#include "serial.h"

#define N_SERIAL_CB 64

// USART0
volatile char rxbuffer[N_SERIAL_CB];
volatile char txbuffer[N_SERIAL_CB];

volatile uint8_t serial_rx_read_index = 0;
volatile uint8_t serial_rx_write_index = 0;

volatile uint8_t serial_tx_read_index = 0;
volatile uint8_t serial_tx_write_index = 0;

// USART1
volatile char rxbuffer2[N_SERIAL_CB];
volatile char txbuffer2[N_SERIAL_CB];

volatile uint8_t serial2_rx_read_index = 0;
volatile uint8_t serial2_rx_write_index = 0;

volatile uint8_t serial2_tx_read_index = 0;
volatile uint8_t serial2_tx_write_index = 0;

uint16_t get_ubrr(uint32_t baud_rate)
{
    uint16_t ubrr = 0;
    uint32_t ubrr10 = 0;

    ubrr10 = F_CPU * 10 / (16 * baud_rate) - 1;
    ubrr = F_CPU / (16 * baud_rate) - 1;

    // truncate as floor or ceil
    if (abs(ubrr10 - ubrr * 10) > abs(ubrr10 - (ubrr + 1) * 10))
    {
        // closer to ubrr+1
        ubrr += 1;
    }

    return ubrr;
}

void USART0_init(uint32_t baud_rate)
{
    uint16_t baud_val = get_ubrr(baud_rate);

    UCSR0B = (1 << RXEN0) | (1 << TXEN0);                  // Turn on the transmission and reception circuitry
    UCSR0C = (1 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes

    UBRR0H = (baud_val >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
    UBRR0L = baud_val;        // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

    UCSR0B |= (1 << RXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC)
}

void USART1_init(uint32_t baud_rate)
{
    uint16_t baud_val = get_ubrr(baud_rate);

    UCSR1B = (1 << RXEN1) | (1 << TXEN1);                  // Turn on the transmission and reception circuitry
    UCSR1C = (1 << USBS1) | (1 << UCSZ10) | (1 << UCSZ11); // Use 8-bit character sizes

    UBRR1H = (baud_val >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
    UBRR1L = baud_val;        // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

    UCSR1B |= (1 << RXCIE1); // Enable the USART Recieve Complete interrupt (USART_RXC)
}

void USART0_transmit(char data)
{
    /* asteapta pana bufferul e gol */
    while (!(UCSR0A & (1 << UDRE0)))
        ;

    /* pune datele in buffer; transmisia va porni automat in urma scrierii */
    UDR0 = data;
}

void USART0_transmitq(char data)
{
    // circular add
    circular_add(txbuffer, &serial_tx_write_index, data);
}

void USART1_transmit(char data)
{
    /* asteapta pana bufferul e gol */
    while (!(UCSR1A & (1 << UDRE1)))
        ;

    /* pune datele in buffer; transmisia va porni automat in urma scrierii */
    UDR1 = data;
}

void USART1_transmitq(char data)
{
    // circular add
    circular_add(txbuffer2, &serial2_tx_write_index, data);
}

void circular_add(volatile char *buffer, volatile uint8_t *index, char data)
{
    // circular add

    buffer[*index] = data;

    if (*index < N_SERIAL_CB)
    {
        *index += 1;
    }
    else
    {
        *index = 0;
    }
}

char circular_get(volatile char *buffer, volatile uint8_t *read_index, volatile uint8_t *write_index)
{
    // circular get
    if (*read_index != *write_index)
    {
        char c = buffer[*read_index];
        if (*read_index < N_SERIAL_CB)
        {
            *read_index += 1;
        }
        else
        {
            *read_index = 0;
        }
        return c;
    }
    else
    {
        return 0;
    }
}

char USART0_receive()
{
    /* asteapta cat timp bufferul e gol */
    while (!(UCSR0A & (1 << RXC0)))
        ;

    /* returneaza datele din buffer */
    return UDR0;
}

char USART1_receive()
{
    /* asteapta cat timp bufferul e gol */
    while (!(UCSR1A & (1 << RXC1)))
        ;

    /* returneaza datele din buffer */
    return UDR1;
}

uint8_t USART0_available()
{
    return serial_rx_read_index != serial_rx_write_index;
}

uint8_t USART1_available()
{
    return serial2_rx_read_index != serial2_rx_write_index;
}

char USART0_receive_buffered()
{
    return circular_get(rxbuffer, &serial_rx_read_index, &serial_rx_write_index);
}

char USART1_receive_buffered()
{
    return circular_get(rxbuffer2, &serial2_rx_read_index, &serial2_rx_write_index);
}

void USART0_print(const char *data)
{
    while (*data != '\0')
        USART0_transmit(*data++);
}

void USART1_print(const char *data)
{
    while (*data != '\0')
        USART1_transmit(*data++);
}

void USART0_printq(const char *data)
{
    while (*data != '\0')
        USART0_transmitq(*data++);

    UCSR0B |= (1 << UDRIE0);
}

void USART1_printq(const char *data)
{
    while (*data != '\0')
        USART1_transmitq(*data++);

    UCSR1B |= (1 << UDRIE1);
}

/* receive q */
ISR(USART0_RX_vect)
{
    char data = UDR0;
    circular_add(rxbuffer, &serial_rx_write_index, data);
}

/* receive q */
ISR(USART1_RX_vect)
{
    char data = UDR1;
    circular_add(rxbuffer2, &serial2_rx_write_index, data);
}

/* transmit q, not working properly */
ISR(USART0_UDRE_vect)
{
    // circular get
    char data = circular_get(txbuffer, &serial_tx_read_index, &serial_tx_write_index);

    if (!data)
    {
        UCSR0B &= ~(1 << UDRIE0); // disable UDRE interrupt
    }
    else
    {
        UDR0 = data;
    }
}