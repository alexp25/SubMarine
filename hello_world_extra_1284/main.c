#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "serial_parser_c.h"
#include "ads1115.h"

#define DDR_TEST DDRA
#define PORT_TEST PORTA
#define PIN_TEST PA6
#define PIN_SPK PA7

void gpio_init()
{
    DDR_TEST |= (1 << PIN_TEST) | (1 << PIN_SPK);
    PORT_TEST &= ~(1 << PIN_TEST) & ~(1 << PIN_SPK);
}

void setup()
{
    cli();
    // you can also DISABLE JTAG VIA FUSE to enable PC4, PC5

    MCUCR |= (1 << JTD); // Disable JTAG
    MCUCR |= (1 << JTD); // Disable JTAG

    MCUSR = 0;

    gpio_init();

    USART0_init(115200);

    USART0_print("hello world\r\n");

    sei();

    for (uint8_t i = 0; i < 2; i++)
    {
        PORT_TEST |= (1 << PIN_TEST);
        _delay_ms(500);
        PORT_TEST &= ~(1 << PIN_TEST);
        _delay_ms(500);
    }
}

void onparse(int cmd, long *data, int ndata)
{

    if (ndata == -1)
    {
        // checksum error
        // send error message
        return;
    }
    switch (cmd)
    {
    case 1:
        // action 1
        // power motors
        // motor1 = data[1]; motor2 = data[2]
        // data[0] is cmd
        break;
    }
}

void check_adc_module()
{
  uint16_t adc_val[4];

  uint8_t ready = sweep_read_noblock(adc_val);
  if (ready)
  {

    // resistor divider R1 = 6k8, R2 = 1k
    bat1 = to_volts(adc_val[0]) * 7.8;    
    // avem doar o baterie
    // bat2 = to_volts(adc_val[1]) * 7.8;
  }
}

void loop()
{

    // check for incoming data via USART0
    check_com(&onparse);
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