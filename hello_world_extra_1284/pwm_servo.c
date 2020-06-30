#include "pwm_servo.h"

uint8_t *port_servo[N_SERVO_MAX];
uint8_t pin_servo[N_SERVO_MAX];
uint16_t pwm_servo[N_SERVO_MAX];

volatile uint8_t channel_max;
volatile uint8_t channel_crt;
volatile uint8_t servo_cycle;

void servo_init_setup()
{
    for (uint8_t i = 0; i < N_SERVO_MAX; i++)
    {
        pwm_servo[i] = 2000;
    }

    channel_max = 0;
    channel_crt = 0;
    servo_cycle = 0;
}

void servo_init_ctrl()
{

    TCNT1 = 0;
    // TOP 50Hz
    OCR1A = 40000;
    OCR1B = 1000;

    // 0.5 us resolution => 2000 steps
    TCCR1A = 0;
    TCCR1B = 0;

    // mod de functionare in CTC cu TOP la ICR1
    // TCCR1B |= (1 << WGM12) | (1 << WGM13);
    // mod de functionare CTC
    TCCR1B |= (1 << WGM12);
    // mod de functionare Fast PWM cu TOP la ICR1
    // TCCR1B |= (1 << WGM12) | (1 << WGM13);
    // TCCR1A |= (1 << WGM11);

    // prescaler 8
    TCCR1B |= (1 << CS11);

    // activeaza intreruperea de comparare cu OCR1A si OCR1B
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);

    // activeaza intreruperea de ICR (capt).
    // TIMSK1 |= (1 << ICIE1);

    // TIMSK1 |= (1 << TOIE1); // enable Timer 1 overflow interrupt
}

uint8_t attach_servo(volatile uint8_t *port, uint8_t pin)
{
    if (channel_max < N_SERVO_MAX - 1)
    {
        port_servo[channel_max] = (uint8_t *)port;
        pin_servo[channel_max] = pin;
        pwm_servo[channel_max] = 0;
        channel_max += 1;
        return 1;
    }
    return 0;
}

uint16_t us_to_ticks(uint16_t us)
{
    return us * 2;
}

uint16_t ticks_to_us(uint16_t ticks)
{
    return ticks / 2;
}

void servo_set_cmd(uint8_t channel, uint16_t us)
{
    if (channel < channel_max)
    {
        pwm_servo[channel] = us_to_ticks(us);
    }
}

void servo_set_state(uint8_t channel, uint8_t state)
{
    if (channel < channel_max)
    {
        if (!state)
        {
            (*port_servo[channel]) &= ~(1 << pin_servo[channel]);
        }
        else
        {
            (*port_servo[channel]) |= (1 << pin_servo[channel]);
        }
    }
}

void start_cycle()
{

    // for (uint8_t i = 0; i < channel_max; i++)
    // {
    //     // clear all outputs (safety)
    //     servo_set_state(i, 0);
    // }

    // PORT_TEST |= (1 << PIN_SPK);

    if (channel_max > 0)
    {
        // start a new 20ms cycle
        servo_set_state(0, 1);
        OCR1B = pwm_servo[0];

        channel_crt = 1;
        servo_cycle = 1;
    }
}

ISR(TIMER1_COMPA_vect)
{
    start_cycle();
}

/* intreruperea de comparatie cu OCR1A */
ISR(TIMER1_COMPB_vect)
{
    if (servo_cycle)
    {

        // PORT_TEST &= ~(1 << PIN_SPK);
        // servo_set_state(0, 0);

        // clear prev servo
        if (channel_crt > 0)
        {
            servo_set_state(channel_crt - 1, 0);
        }

        if (channel_crt < channel_max)
        {
            // activate crt servo
            if (pwm_servo[channel_crt] != 0)
            {
                servo_set_state(channel_crt, 1);
            }
            uint16_t next_timeout = pwm_servo[channel_crt];
            if (next_timeout == 0)
            {
                next_timeout = 1000;
            }
            // set OCR1B
            OCR1B = TCNT1 + next_timeout;
            // set next channel afterwards
            channel_crt += 1;
        }
        else
        {
            // clear last servo
            if (channel_max > 0)
            {
                servo_set_state(channel_max - 1, 0);
            }
            // start over
            channel_crt = 0;
            // first wait for next period
            servo_cycle = 0;
        }
        // PORT_TEST ^= (1 << PIN_SPK);
    }

    //  servo_set_state(channel_crt - 1, 0);
}

/* this can be used for top ICR1 interrupt */
// ISR(TIMER1_CAPT_vect)
// {
// }

// ISR(TIMER1_OVF_vect)
// {
//     start_cycle();
// }