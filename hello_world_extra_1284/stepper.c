#include "stepper.h"

int8_t stepper_step = 0, stepper_direction = 1;

void init_stepper()
{
    DDRS1 |= (1 << PINS1);
    DDRS2 |= (1 << PINS2);
    DDRS3 |= (1 << PINS3);
    DDRS4 |= (1 << PINS4);

    PORTS1 &= ~(1 << PINS1);
    PORTS2 &= ~(1 << PINS2);
    PORTS3 &= ~(1 << PINS3);
    PORTS4 &= ~(1 << PINS4);
}

void micro_step()
{
    switch (stepper_step)
    {
    case 0:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 |= (1 << PINS4);
        break;
    case 1:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 |= (1 << PINS3);
        PORTS4 |= (1 << PINS4);
        break;
    case 2:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 |= (1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 3:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 |= (1 << PINS2);
        PORTS3 |= (1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 4:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 |= (1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 5:
        PORTS1 |= (1 << PINS1);
        PORTS2 |= (1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 6:
        PORTS1 |= (1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 7:
        PORTS1 |= (1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 |= (1 << PINS4);
        break;
    default:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
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
        PORTS1 &= ~(1 << PINS1);
        PORTS2 |= (1 << PINS2);
        PORTS3 |= (1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 1:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 |= (1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 |= (1 << PINS4);
        break;
    case 2:
        PORTS1 |= (1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 |= (1 << PINS4);
        break;
    case 3:
        PORTS1 |= (1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 |= (1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    default:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
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
        PORTS1 |= (1 << PINS1);
        PORTS2 |= (1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 1:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 |= (1 << PINS2);
        PORTS3 |= (1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    case 2:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 |= (1 << PINS3);
        PORTS4 |= (1 << PINS4);
        break;
    case 3:
        PORTS1 |= (1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 |= (1 << PINS4);
        break;
    default:
        PORTS1 &= ~(1 << PINS1);
        PORTS2 &= ~(1 << PINS2);
        PORTS3 &= ~(1 << PINS3);
        PORTS4 &= ~(1 << PINS4);
        break;
    }
    stepper_set_dir(4);
}
#endif

void stepper_stop()
{
    PORTS1 &= ~(1 << PINS1);
    PORTS2 &= ~(1 << PINS2);
    PORTS3 &= ~(1 << PINS3);
    PORTS4 &= ~(1 << PINS4);
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