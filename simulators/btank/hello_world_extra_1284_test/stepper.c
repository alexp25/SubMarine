#include "stepper.h"

int8_t stepper_step = 0, stepper_direction = 1;

void init_stepper()
{
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
    PORTB &= ~(1 << PB0) & ~(1 << PB1) & ~(1 << PB2) & ~(1 << PB3);
}

void micro_step()
{
    switch (stepper_step)
    {
    case 0:
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB |= (1 << PB3);
        break;
    case 1:
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB |= (1 << PB2);
        PORTB |= (1 << PB3);
        break;
    case 2:
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB |= (1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 3:
        PORTB &= ~(1 << PB0);
        PORTB |= (1 << PB1);
        PORTB |= (1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 4:
        PORTB &= ~(1 << PB0);
        PORTB |= (1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 5:
        PORTB |= (1 << PB0);
        PORTB |= (1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 6:
        PORTB |= (1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 7:
        PORTB |= (1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB |= (1 << PB3);
        break;
    default:
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    }
    stepper_set_dir(8);
}

#ifdef STEPPER_DEMO
void full_step()
{
    switch (stepper_step)
    {
    case 0:
        PORTB &= ~(1 << PB0);
        PORTB |= (1 << PB1);
        PORTB |= (1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 1:
        PORTB &= ~(1 << PB0);
        PORTB |= (1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB |= (1 << PB3);
        break;
    case 2:
        PORTB |= (1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB |= (1 << PB3);
        break;
    case 3:
        PORTB |= (1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB |= (1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    default:
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    }
    stepper_set_dir(4);
}
#endif
#ifndef STEPPER_DEMO
void full_step()
{
    switch (stepper_step)
    {
    case 0:
        PORTB |= (1 << PB0);
        PORTB |= (1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 1:
        PORTB &= ~(1 << PB0);
        PORTB |= (1 << PB1);
        PORTB |= (1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    case 2:
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB |= (1 << PB2);
        PORTB |= (1 << PB3);
        break;
    case 3:
        PORTB |= (1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB |= (1 << PB3);
        break;
    default:
        PORTB &= ~(1 << PB0);
        PORTB &= ~(1 << PB1);
        PORTB &= ~(1 << PB2);
        PORTB &= ~(1 << PB3);
        break;
    }
    stepper_set_dir(4);
}
#endif

void stepper_stop()
{
    PORTB &= ~(1 << PB0);
    PORTB &= ~(1 << PB1);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);
}

void stepper_set_dir(int nsteps)
{
    if (stepper_direction == 1)
    {
        stepper_step++;
        if (stepper_step > nsteps - 1)
        {
            stepper_step = 0;
        }
    }
    else
    {
        stepper_step--;
        if (stepper_step < 0)
        {
            stepper_step = nsteps - 1;
        }
    }
}