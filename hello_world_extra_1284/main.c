#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "serial_parser_c.h"
#include "mpu6050_v2.h"
#include "twi_arduino.h"
#include "pwm_servo.h"
#include "math_utils.h"
#include "ads1115.h"

#define DDR_TEST DDRA
#define PORT_TEST PORTA
#define PIN_TEST PA6
#define PIN_SPK PA7

#define NORMAL_MODE 0
#define ASSISTED_SINK_MODE 1
/**
 * Opperation mode for the submarine:
 * 0 - normal
 * 1 - assisted sink 
 */
char opperation_mode;
char sink_angle;
char msg[100];

double bat1;
double current;
volatile float err_roll,err_pitch, err_yaw;

#define N_MOTORS 1
#define N_SERVOS 2
int poz_servos[N_SERVOS];
int poz_motors[N_MOTORS];

void gpio_init()
{
    DDR_TEST |= (1 << PIN_TEST) | (1 << PIN_SPK);
    PORT_TEST &= ~(1 << PIN_TEST) & ~(1 << PIN_SPK);
}

void init_mpu_timer()
{
    //250Hz
    OCR0A = 124;
    TCCR0A = 0;
    TCNT0 = 0;
    //CTC with TOP at OCR0A
    TCCR0A |= (1<<WGM01); 
    //prescaler 256
    TCCR0B |= (1<<CS02);
    //enable the interrupt
    TIMSK0 = (1 << OCIE1A);
    USART0_print("done initializing timer\r\n");
}

void init_adc()
{
    DDRA &= ~(1<<PA0);

    ADMUX = 0;
    ADMUX |= (1 << REFS0);

    ADCSRA = 0;
    /* set prescaler at 128 */
    ADCSRA |= (7 << ADPS0);
    /* enable ADC */
    ADCSRA |= (1 << ADEN);
    /*enable the interrupt*/
    ADCSRA |= (1 << ADIE);
}

volatile uint8_t sw_mpu_read_trigger;
volatile uint8_t sw_mpu_write_angles;
/*
 * 0 - idle
 * 1 - read data from twi
 */
volatile uint8_t mpu_state;
volatile uint8_t mpu_write_state;
volatile uint16_t adc_value;

ISR(ADC_vect)
{
    adc_value = ADC;
}

ISR(TIMER0_COMPA_vect)
{

    sw_mpu_read_trigger++;
    sw_mpu_write_angles++;

    if(sw_mpu_read_trigger == 5){
        mpu_state = 1;
        sw_mpu_read_trigger = 0;
    }

    if(sw_mpu_write_angles==250) {
        PORT_TEST ^= (1<<PIN_TEST);
        mpu_write_state = 1;
        sw_mpu_write_angles=0;
    }
}

int servo_pos=500;
int esc_pos=1000;
/*
 * 0 - aripi
 * 1 - carma
 */
void initialize_servos()
{
    servo_init_setup();
    DDRD |= (1 << PD4);
    DDRD |= (1 << PD5);
    DDRD |= (1 << PD6);
    attach_servo(&PORTD, PD4); //ze flaps
    attach_servo(&PORTD, PD5); //ze carma
    attach_servo(&PORTD, PD6); //ze elice
    servo_init_ctrl();
    servo_set_cmd(0, servo_pos);
    servo_set_cmd(1, servo_pos);
    servo_set_cmd(2, esc_pos);
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

    wire_begin();
    mpu6050_config();

    mp6050_initialize_errors();
    //mp6050_read_errors();

    init_mpu_timer();

    initialize_servos();
    init_ads1115();
    init_adc();

    opperation_mode = NORMAL_MODE;

}

#define CARMA 7
#define SINK_ANGLE 6
#define WING_FLAPS 2

#define OUT_MOTOR_DATA 1
#define OUT_ACK 200
#define OUT_NACK 400
#define OUT_STATUS 3
#define OUT_SENSOR_DATA 4
#define OUT_SETTINGS 5
void append_float(char *dest, float data) {
    int nr = (int)data;
    
    if(data < 0)
        sprintf(msg,",%d.%d",nr, nr*100 - (int)(data*100));
    else
        sprintf(msg,",%d.%d",nr, (int)(data*100) - nr*100);

    strcat(dest,msg);
}

void append_command(char *dest, int data) {
    sprintf(msg,"%d",data);
    strcat(dest, msg);
}

void append_int(char *dest, int data) {
    sprintf(msg,",%d",data);
    strcat(dest, msg);
}

void send_sensors_data()
{
    char mesaj[64];
    mesaj[0]=0;

    append_command(mesaj,OUT_SENSOR_DATA);

    append_int(mesaj,(int)mp_roll);
    append_int(mesaj,(int)mp_pitch);
    append_int(mesaj,(int)mp_yaw);
    append_float(mesaj,0);
    append_float(mesaj,0);
    append_float(mesaj,0);
    USART0_print(mesaj);
    mesaj[0]=0;
    append_float(mesaj,0);
    append_float(mesaj,current);
    append_float(mesaj,0);
    append_float(mesaj,0);
    append_float(mesaj,0);
    append_float(mesaj,0);
    append_float(mesaj,0);
    append_float(mesaj,bat1);
    append_float(mesaj,0);
    strcat(mesaj,"\n");
    USART0_print(mesaj);
}

void send_motors_data()
{
    char mesaj[64];
    uint8_t i;
    mesaj[0]=0;
    append_command(mesaj,OUT_MOTOR_DATA);

    append_int(mesaj, N_MOTORS);
    append_int(mesaj, N_SERVOS);
    for( i = 0; i < N_MOTORS; i++)
        append_float(mesaj, ((float)poz_motors[i] - 1000)/2000 ) ;

    for( i = 0; i < N_SERVOS; i++)
        append_float(mesaj, ((float)poz_servos[i]- 500)/2200 );

    strcat(mesaj,"\n");
    USART0_print(mesaj);
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
    case WING_FLAPS:
        poz_servos[0] =  data[1]*17/2 + 1350;
        servo_set_cmd(0,poz_servos[0]);
        break;
    case SINK_ANGLE:
        sink_angle = data[2];
        break;
    case CARMA:
        poz_servos[1] =  data[2]*17/2 + 1350;
        poz_motors[0] = (data[1]<0?0:data[1])*5 + 1000;
        servo_set_cmd(1, poz_servos[1]);
        servo_set_cmd(2, poz_motors[0]);
        break;
    case 202:
        send_sensors_data();
        break;
    case 203:
        send_motors_data();
        break;
    }
}

void check_adc_module()
{
    uint16_t adc_val;

    uint8_t ready = sweep_read_noblock(&adc_val);
    if (ready)
        // resistor divider R1 = 6k8, R2 = 1k
        bat1 = 0.9*bat1 + 0.1*(to_volts(adc_val) * 7.8);    
}

void read_adc()
{
    /* start conversion */
    ADCSRA |= (1 << ADSC);
}

int nr;

void loop()
{
    // check for incoming data via USART0

    check_com(&onparse);
    
    if( mpu_state == 1) 
    {
        check_adc_module();
        read_adc();
        current = 0.9* current + (float)adc_value*7.33/1024 - 3.67;
        //current = 0.9 * current + 0.1 * ( (float)adc_value *5 / 1024);
        mpu6050_v2_read();
        mp6050_correct_errors();
        mp6050_compute_angles();
        mpu_state = 0;
        if(nr<100)
        {
            err_roll += mp_roll;
            err_pitch += mp_pitch;
            err_yaw += mp_yaw;
            nr++;
        }
        if(nr==100){
            USART0_print("fac treabaaaa\r\n");
            err_roll /=100;
            err_pitch /=100;
            err_yaw /=100;
            nr++;
        }
    }
    

    if( mpu_write_state == 1) {
        mp_roll = mp_roll - err_roll;
        mp_pitch = mp_pitch - err_pitch;
        mp_yaw = mp_yaw - err_yaw;
        sprintf(msg,"%.3f  %.3f  %.3f   %.3f %.3f %.3f\r\n",mp_roll,mp_pitch,mp_yaw,err_roll, err_pitch, err_yaw);
        USART0_print(msg);        
        mpu_write_state = 0;
    }

    if(opperation_mode == ASSISTED_SINK_MODE) {
        //add logic for assisted sink
    }
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