
#ifndef __TIME_UTILS_H__
#define __TIME_UTILS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

extern volatile uint32_t millis_counter;
extern volatile uint32_t micros_counter;

 void setup_timer();
 uint32_t millis();
 uint32_t micros();

#ifdef __cplusplus
}
#endif

#endif
