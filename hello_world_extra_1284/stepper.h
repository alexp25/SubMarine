#ifndef __STEPPER_H__
#define __STEPPER_H__

#ifdef __cplusplus
extern "C"
{
#endif

// #define STEPPER_DEMO 1

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// pc5 (esc2), pc7 (esc4), pd4 (s1), pd5 (s2)
#define DDRS1 DDRC
#define DDRS2 DDRC
#define DDRS3 DDRD
#define DDRS4 DDRD

#define PORTS1 PORTC
#define PORTS2 PORTC
#define PORTS3 PORTD
#define PORTS4 PORTD

#define PINS1 PC5
#define PINS2 PC7
#define PINS3 PD4
#define PINS4 PD5

    extern int8_t stepper_step, stepper_direction;
    void init_stepper();
    void micro_step();
    void full_step();
    void stepper_stop();
    void stepper_set_dir(int nsteps);

#ifdef __cplusplus
}
#endif

#endif