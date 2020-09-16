#ifndef __STEPPER_H__
#define __STEPPER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define STEPPER_DEMO 1

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

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