#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "serial_parser_c.h"
#include "mpu9250_v2.h"
#include "twi_arduino.h"
#include "pwm_servo.h"
#include "math_utils.h"
#include "ads1115.h"
#include "eeprom_config.h"
#include "main.h"

#define DDR_TEST DDRA
#define PORT_TEST PORTA
#define PIN_TEST PA6
#define PIN_SPK PA7

#define TRIGGER_PIN PB0
#define ECHO_PIN PB1

#define NORMAL_MODE 0
#define ASSISTED_SINK_MODE 1
#define RETURN_HOME 2
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
float roll,pitch,yaw;

//set to one if you want to calibrate the magnetometer
int calibrate_magnetometer = 0;

//limits for the servos
int servo_carma_mid_position = 1500;
int servo_carma_upper_limit = servo_carma_mid_position + 300;
int servo_carma_lower_limit = servo_carma_mid_position - 300;

int servo_wings_mid_position = 1500;
int servo_wings_upper_limit = servo_wings_mid_position + 300;
int servo_wings_lower_limit = servo_wings_mid_position - 300;


#define N_MOTORS 1
#define N_SERVOS 2
int poz_servos[N_SERVOS];
int poz_motors[N_MOTORS];

void gpio_init()
{
    DDR_TEST |= (1 << PIN_TEST) | (1 << PIN_SPK);
    PORT_TEST &= ~(1 << PIN_TEST) & ~(1 << PIN_SPK);

    DDRB |= (1 << TRIGGER_PIN);
    PORTB &= ~(1 << TRIGGER_PIN);
    DDRB &= ~(1 << ECHO_PIN); 
    PORTB &= ~(1 << ECHO_PIN);

    //interrupts on the echo pin
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT9);
}

void init_mpu_timer()
{
    //250Hz
    OCR0A = 249;
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    //CTC with TOP at OCR0A
    TCCR0A |= (1 << WGM01);
    //prescaler 256
    TCCR0B |= (1 << CS02);
    //enable the interrupt
    TIMSK0 = (1 << OCIE0A);
}

void init_sonar_timer()
{
    OCR2A = 249;
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    //prescaler 32
    TCCR2B = ( 1 << CS21) | ( 1 << CS20);
    //CTC with TOP at OCR0A
    TCCR2A |= (1<<WGM21);
    //enable the interrupt
    TIMSK2 = (1 << OCIE2A);
}


volatile int sonar_time;
volatile double sonar_distance;
volatile char sonar_wait;

ISR(TIMER2_COMPA_vect) //one interrupt per milisecond
{
    if( PINB & ( 1 << ECHO_PIN)) {
        sonar_time+=500;
    }
}

ISR(PCINT1_vect){
  
  //daca trece pe 1 incepe o masuratoare
  if ( PINB & (1 << ECHO_PIN) ){
    sonar_time = -TCNT2<<1;
    //prescaler 32
    //TCCR2B = ( 1 << CS21) | ( 1 << CS20);
  } else { //altfel o termina
    sonar_time += TCNT2<<1; //16Mhz , prescaler 32 => 250kHz => 10^6/25*10^4 = 4 us per tick
    //TCCR2B = 0;
    sonar_wait = 0;
  }
}

void init_adc()
{
    DDRA &= ~(1 << PA0);

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
volatile uint8_t sw_sonar_activation;
/*
 * 0 - idle
 * 1 - read data from twi
 */
volatile uint8_t mpu_state;
volatile uint8_t sonar_activated;
volatile uint16_t adc_value;
volatile uint8_t check_connection;

ISR(ADC_vect)
{
    adc_value = ADC;
}

ISR(TIMER0_COMPA_vect)
{

    sw_mpu_read_trigger++;
    sw_sonar_activation++;

    if (sw_mpu_read_trigger == 5)
    {
        mpu_state = 1;
        check_connection = 1;
        sw_mpu_read_trigger = 0;
    }

    if(sw_sonar_activation==250) {
        PORT_TEST ^= (1<<PIN_TEST);
        sonar_activated = 1;
        sw_sonar_activation=0;
    }
}


int servo_pos=1000;
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
    mpu9250_v2_init();
    mpu9250_initialize_errors();

    init_mpu_timer();
    init_sonar_timer();

    initialize_servos();
    init_ads1115();
    init_adc();

    pid_initialize_errors();    

    opperation_mode = NORMAL_MODE;
}

void append_float(char *dest, float data)
{
    int nr = (int)data;

    if (data < 0)
        sprintf(msg, ",%d.%d", nr, nr * 100 - (int)(data * 100));
    else
        sprintf(msg, ",%d.%d", nr, (int)(data * 100) - nr * 100);

    strcat(dest, msg);
}

void append_command(char *dest, int data)
{
    sprintf(msg, "%d", data);
    strcat(dest, msg);
}

void append_int(char *dest, int data)
{
    sprintf(msg, ",%d", data);
    strcat(dest, msg);
}

void send_sensors_data()
{
    char mesaj[64];
    mesaj[0] = 0;

    append_command(mesaj, OUT_SENSOR_DATA);

    append_int(mesaj,(int)roll);
    append_int(mesaj,(int)pitch);
    append_int(mesaj,(int)yaw);
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
    append_float(mesaj,yaw);
    append_float(mesaj,sonar_distance);
    append_float(mesaj,bat1);
    append_float(mesaj,0);
    strcat(mesaj,"\n");
    USART0_print(mesaj);
}

void send_motors_data()
{
    char mesaj[64];
    uint8_t i;
    mesaj[0] = 0;
    append_command(mesaj, OUT_MOTOR_DATA);

    append_int(mesaj, N_MOTORS);
    append_int(mesaj, N_SERVOS);
    for (i = 0; i < N_MOTORS; i++)
        append_float(mesaj, ((float)poz_motors[i] - 1000) / 2000);

    for (i = 0; i < N_SERVOS; i++)
        append_float(mesaj, ((float)poz_servos[i] - 500) / 2200);

    strcat(mesaj, "\n");
    USART0_print(mesaj);
}


int lost_connection_counter;
char msg_received;

float starting_direction = 500;
struct pid_context return_home_context;
float dt = 0.02;
int motor_return_power = 1500;

inline int limit_servo(int x, int upper, int lower)
{
    if( x > upper)
        return upper;
    if( x < lower)
        return lower;
    return x;
}

void onparse(int cmd, long *data, int ndata)
{
    msg_received = 1;
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
        poz_servos[0] = limit_servo(data[1] * 17 / 2 + servo_wings_mid_position, servo_wings_upper_limit, servo_wings_lower_limit);
        servo_set_cmd(0, poz_servos[0]);
        break;
    case SINK_ANGLE:
        sink_angle = data[2];
        break;
    case CARMA:
        poz_servos[1] = limit_servo(data[2] * 17 / 2 + servo_carma_mid_position, servo_carma_upper_limit, servo_carma_lower_limit);
        poz_motors[0] = (data[1] < 0 ? 0 : data[1]) * 10 + esc_pos;

        if( poz_motors[0] > 0 && starting_direction == 500)
            starting_direction = yaw;
        servo_set_cmd(1, poz_servos[1]);
        servo_set_cmd(2, poz_motors[0]);
        break;
    case 202:
        send_sensors_data();
        break;
    case 203:
        send_motors_data();
        break;
    case READ_MPU:
        mpu9250_initialize_errors();
        break;
    case WRITE_MPU_A:
        mpu9250_write_a_errors( data[1], data[2], data[3]);
        break;
    case WRITE_MPU_G:
        mpu9250_write_g_errors( data[1], data[2], data[3]);
        break;
    case WRITE_MPU_M:
        mpu9250_write_m_errors( data[1], data[2], data[3]);
        break;
    case READ_PID:
        pid_initialize_errors();
        break;
    case WRITE_PID:
        pid_write_coefficients( data[1], data[2], data[3]);
        break;
    case WRITE_SENTINEL:
        write_sentinel((Sentinel_Device) data[1], EEPROM_SENTINEL);
        break;
    case CLEAR_SENTINEL:
        write_sentinel((Sentinel_Device) data[1], EEPROM_DEFAULTS);
        break;
    case READ_SERVOS:
        servo_initialize_bias(data[1]);
        break;
    case WRITE_SERVOS:
        servo_write_bias(data[1], data[2]);
        break;
    }
}

void check_adc_module()
{
    uint16_t adc_val;

    uint8_t ready = read_noblock_channel(&adc_val, 0);
    if (ready)
        // resistor divider R1 = 6k8, R2 = 1k
        bat1 = 0.9 * bat1 + 0.1 * (to_volts(adc_val) * 7.8);
}

void read_adc()
{
    /* start conversion */
    ADCSRA |= (1 << ADSC);
}

void compute_sonar_distance() {
    if( sonar_wait == 0)
        sonar_distance = sonar_time * 0.01715;
        sonar_wait = 1;        
        PORTB |= (1 << TRIGGER_PIN);
        _delay_us(10);
        PORTB &= ~(1 << TRIGGER_PIN);
}
int nr;

//function that return the submarine home
void return_home()
{
    //compute the angle of ze carma so that the boat will return home
    poz_servos[1] = limit_servo( update_pid(&return_home_context, yaw), servo_carma_upper_limit, servo_wings_lower_limit);
    servo_set_cmd(1, poz_servos[1]);
}

void loop()
{

    // check for incoming data via USART0
    if(calibrate_magnetometer)
    {
        if(nr == 0)
                USART0_print("calibrating magnetometer\r\n");
        
        if(mpu_state == 1) {
            mpu9250_readMagData();
            mpu9250_calibrate();
            nr++;

            
            if( nr % 50 == 0)
                {
                    sprintf(msg,"%d\r\n",nr/50);
                    USART0_print(msg);
                }
            if(nr == 1500)
            {
                USART0_print("done calibration\r\n");
                mpu9250_print_calib();
                calibrate_magnetometer = 0;
                nr = 0;
            }

            mpu_state = 0;
        }
        return;
    }

    check_com(&onparse);

    if (mpu_state == 1)
    {
        check_adc_module();
        read_adc();
        current = 0.9 * current + (float)adc_value * 7.33 / 1024 - 3.67;
        //current = 0.9 * current + 0.1 * ( (float)adc_value *5 / 1024);
        mpu9250_v2_read();
        mpu9250_readMagData();
        mpu9250_correct_errors();
        mpu9250_compute_angles();
        
        roll = mp_roll;
        pitch = mp_pitch;
        yaw = mp_yaw;

        mpu_state = 0;
        if(nr<200)
        {
            err_roll += roll;
            err_pitch += pitch;
            //err_yaw += yaw;
            nr++;
        } else {
            roll -= err_roll;
            pitch -= err_pitch;
            //yaw -= err_yaw;        
        }

        if(nr==200){
            err_roll /=200;
            err_pitch /=200;
            //err_yaw /=200;
            nr++;
        }
        else
        {
            roll = roll - err_roll;
            pitch = pitch - err_pitch;
            //yaw = yaw - err_yaw;
        }
    }
    
    if( sonar_activated == 1) {
        //sprintf(msg,"%d %d %d %.3f\r\n",mpu9250_magX, mpu9250_magY, mpu9250_magZ, yaw);
        //USART0_print(msg);
        compute_sonar_distance();
        sonar_activated = 0;
    }

    if(check_connection == 1) {
        check_connection = 0;

        if( msg_received == 1) {
            msg_received = 0;
            lost_connection_counter = 0;
            
        } else if(opperation_mode != RETURN_HOME){
            lost_connection_counter ++;
        }
        
        if(lost_connection_counter == 100)
        {
            lost_connection_counter++;
            opperation_mode = RETURN_HOME;
            if(return_home_context.dt != 0)
            {
                initialize_pid_contex(&return_home_context, dt, starting_direction);
                load_weights(&return_home_context, kp, ki, kd);
                servo_set_cmd(2, poz_motors[motor_return_power]); //set a low speed for returning home
            }
        }
    }

    if(opperation_mode == RETURN_HOME)
        return_home();

    if (opperation_mode == ASSISTED_SINK_MODE)
    {
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