#ifndef __PWM_SERVO_H__
#define __PWM_SERVO_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define N_SERVO_MAX 10

    extern uint8_t *port_servo[N_SERVO_MAX];
    extern uint8_t pin_servo[N_SERVO_MAX];
    extern uint16_t pwm_servo[N_SERVO_MAX];

    extern volatile uint8_t channel_max;
    extern volatile uint8_t channel_crt;
    extern volatile uint8_t servo_cycle;

    void servo_init_setup();
    void servo_init_ctrl();
    uint8_t attach_servo(volatile uint8_t *port, uint8_t pin);
    uint16_t us_to_ticks(uint16_t us);
    uint16_t ticks_to_us(uint16_t ticks);
    void start_cycle();
    void servo_set_cmd(uint8_t channel, uint16_t us);
    void servo_set_state(uint8_t channel, uint8_t state);

#ifdef __cplusplus
}
#endif

#endif