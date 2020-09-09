#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "serial_parser_c.h"
#include "twi_arduino.h"
#include "math_utils.h"
#include "stepper.h"

#define DDR_TEST DDRA
#define PORT_TEST PORTA
#define PIN_TEST PA6
#define PIN_SPK PA7

volatile uint8_t ps_stepper = 0;
volatile uint8_t disp = 0;

void gpio_init()
{
    DDR_TEST |= (1 << PIN_TEST) | (1 << PIN_SPK);
    PORT_TEST &= ~(1 << PIN_TEST) & ~(1 << PIN_SPK);

    DDRA &= ~(1 << PA0);
    PORTA |= (1 << PA0);

    PCICR |= (1 << PCINT0);
    PCMSK0 |= (1 << PCINT0);
}

uint8_t sim_step = 0;
#define SIM_LENGTH 100
int16_t sim_step_counter = SIM_LENGTH / 2;

void simulator_init()
{
    DDRC = 255;
    PORTC = 0;
    DDRA |= (1 << PA1) | (1 << PA2);
    PORTA &= ~(1 << PA1) & ~(1 << PA2);
}

void simulator_eval()
{

    if (sim_step_counter > SIM_LENGTH)
    {
        PORTA |= (1 << PA2);
        return;
    }
    if (sim_step_counter < 0)
    {
        PORTA |= (1 << PA1);
        return;
    }
    PORTA &= ~(1 << PA1) & ~(1 << PA2);

    sim_step = sim_step_counter * 8 / SIM_LENGTH;
    PORTC = (1 << sim_step);
}

void simulator_step()
{
    sim_step_counter += stepper_direction == 1 ? 1 : -1;
    if (sim_step_counter > SIM_LENGTH + 1)
    {
        sim_step_counter = SIM_LENGTH + 1;
    }
    if (sim_step_counter < -1)
    {
        sim_step_counter = -1;
    }
}

void init_mpu_timer()
{
    //250Hz
    OCR0A = 124;

    //1250Hz
    // OCR0A = 24;

    TCCR0A = 0;
    TCNT0 = 0;
    //CTC with TOP at OCR0A
    TCCR0A |= (1 << WGM01);
    //prescaler 256
    TCCR0B |= (1 << CS02);
    //enable the interrupt
    TIMSK0 = (1 << OCIE1A);
    USART0_print("done initializing timer\r\n");
}

ISR(PCINT0_vect)
{
    if ((PINA & (1 << PA0)) == 0)
    {
        stepper_direction ^= 1;
    }
}

ISR(TIMER0_COMPA_vect)
{
    ps_stepper++;
    if (ps_stepper == 10)
    {
        // should be @200 Hz in real life (max speed for my stepper)
        // tip: frequency controls speed
        ps_stepper = 0;
        full_step();
        simulator_step();
        // micro_step();
        disp = 1;
    }
}

void setup()
{
    cli();
    // you can also DISABLE JTAG VIA FUSE to enable PC4, PC5

    MCUCR |= (1 << JTD); // Disable JTAG
    MCUCR |= (1 << JTD); // Disable JTAG

    MCUSR = 0;

    gpio_init();

    stepper_direction = 0;
    init_stepper();

    init_mpu_timer();

    USART0_init(115200);

    simulator_init();

    USART0_print("hello world\r\n");

    sei();

    // for (uint8_t i = 0; i < 2; i++)
    // {
    //     PORT_TEST |= (1 << PIN_TEST);
    //     _delay_ms(500);
    //     PORT_TEST &= ~(1 << PIN_TEST);
    //     _delay_ms(500);
    // }
}

void loop()
{
    if (disp)
    {
        disp = 0;
        char buf[100];
        sprintf(buf, "step: %d,%d\r\n", sim_step, sim_step_counter);
        USART0_print(buf);

        // TODO 1: start/stop stepper motor via button
        // TODO 2: stop stepper motor when (simulated) limit is reached (LIM1/LIM2), use PCINT
        // TODO 3: print stepper position 0-100
        // TODO 4: set position controller (e.g. want position 30%, stepper has to go there, then stop)
    }

    simulator_eval();
}

int main(void)
{
    setup();
    for (;;)
    {
        loop();
    }
    return 0;
}