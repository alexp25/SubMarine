#ifndef __SERIAL_H__
#define __SERIAL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "math_utils.h"


    // USART 0

    uint16_t get_ubrr(uint32_t baud_rate);
    char circular_get(volatile char *buffer, volatile uint8_t *read_index, volatile uint8_t *write_index);
    void circular_add(volatile char *buffer, volatile uint8_t *index, char data);

    void USART0_init(uint32_t baud_rate);
    void USART0_transmit(char data);
    char USART0_receive();
    void USART0_print(const char *data);
    uint8_t USART0_available();
    char USART0_receive_buffered();

    // USART 1

    void USART1_init(uint32_t baud_rate);
    void USART1_transmit(char data);
    char USART1_receive();
    void USART1_print(const char *data);
    uint8_t USART1_available();
    char USART1_receive_buffered();

#ifdef __cplusplus
}
#endif

#endif