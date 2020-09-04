#include "time_utils.h"

volatile uint32_t millis_counter = 0;
volatile uint32_t micros_counter = 0;

// millis timer
void setup_timer()
{
    TCNT3 = 0;
    OCR3A = 249;

    // 0.5 us resolution => 2000 steps
    TCCR3A = 0;
    TCCR3B = 0;

    // mod de functionare CTC
    TCCR3B |= (1 << WGM31);

    // prescaler 64 => 4 us/tick
    TCCR3B |= (1 << CS31) | (1 << CS30);

    // activeaza intreruperea de comparare cu OCR1A si OCR1B
    TIMSK3 = (1 << OCIE3A);
}

ISR(TIMER3_COMPA_vect)
{
    millis_counter += 1;
}

uint32_t millis()
{
    uint32_t mc;
    cli();
    mc = millis_counter;
    sei();
    return mc;
}

uint32_t micros()
{
    uint32_t mc;
    uint32_t tc;
    cli();
    mc = millis_counter;
    tc = TCNT3;
    sei();
    return mc * 1000 + tc * 4;
}