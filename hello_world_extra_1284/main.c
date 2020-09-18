#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdint.h>

#include "serial_parser_c.h"
#include "mpu9250_v2.h"
#include "twi_arduino.h"
#include "pwm_servo.h"
#include "math_utils.h"
#include "ads1115.h"
#include "main.h"
#include "string_utils.h"
#include "settings.h"
#include "pid.h"
#include "time_utils.h"
#include "stepper.h"

#define DDR_TEST DDRA
#define PORT_TEST PORTA
#define PIN_TEST PA6
#define PIN_SPK PA7

#define TRIGGER_PIN PB0
#define ECHO_PIN PB1

#define PUMP_PWM_PIN PC6

#define NORMAL_MODE 0
#define ASSISTED_SINK_MODE 1
#define RETURN_HOME 2
#define AWAITING_START 3


#define DDR_STEPPER DDRA
#define PORT_STEPPER PORTA
#define STEPPER_PIN PINA
#define STEPPER_0_PIN PA2
#define STEPPER_100_PIN PA3
#define STEPPER_0_PCINT PCINT2
#define STEPPER_100_PCINT PCINT3
#define PCIE_STEPPER PCIE0
#define PCMSK_STEPPER PCMSK0
#define STEPPER_PCINT PCINT0_vect

/**
 * Opperation mode for the submarine:
 * 0 - normal
 * 1 - assisted sink 
 */
char opperation_mode;
char sink_angle;
char msg[100];

uint8_t reset = 0;

double bat1;
double current;
volatile float err_roll, err_pitch, err_yaw;
float roll, pitch, yaw;

//set to one if you want to calibrate the magnetometer
int calibrate_magnetometer = 0;

uint32_t bench = 0, bench_ts = 0, ts_start = 0;

int lost_connection_counter;
char msg_received;

float starting_direction = 0;
struct pid_context return_home_context;
float dt = 0.02;

uint8_t return_home_engaged = 0;
uint8_t set_sail_engaged = 0;

void gpio_init()
{
    DDR_TEST |= (1 << PIN_TEST) | (1 << PIN_SPK);
    PORT_TEST &= ~(1 << PIN_TEST) & ~(1 << PIN_SPK);

    DDRB |= (1 << TRIGGER_PIN);
    PORTB &= ~(1 << TRIGGER_PIN);

    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << ECHO_PIN);

    DDRC |= (1 << PUMP_PWM_PIN);
    PORTC &= ~(1 << PUMP_PWM_PIN);

    DDR_STEPPER &= ~( 1 << STEPPER_0_PIN);
    // enable pull-up for switch
    PORT_STEPPER |= ( 1 << STEPPER_0_PIN);

    DDR_STEPPER &= ~( 1 << STEPPER_100_PIN);
     // enable pull-up for switch
    PORT_STEPPER |= ( 1 << STEPPER_100_PIN);

    //interrupts on the echo pin
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT9);

    //interrupts for the stepper
    PCICR |= ( 1 << PCIE_STEPPER);
    // PCMSK_STEPPER |= ( 1 << STEPPER_0_PIN);
    // PCMSK_STEPPER |= ( 1 << STEPPER_100_PIN);
    PCMSK_STEPPER |= ( 1 << STEPPER_0_PCINT);
    PCMSK_STEPPER |= ( 1 << STEPPER_100_PCINT);
}


void init_mpu_timer()
{
    //1KHz
    OCR0A = 249;
    OCR0B =  0;
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    //CTC with TOP at OCR0A
    TCCR0A |= (1 << WGM01);
    //prescaler 64
    TCCR0B |= (1 << CS01) | ( 1 << CS00);
    //enable the interrupt
    TIMSK0 = (1 << OCIE0A) | ( 1 << OCIE0B);
}

void init_sonar_timer()
{
    OCR2A = 249;
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    //prescaler 32
    TCCR2B = (1 << CS21) | (1 << CS20);
    //CTC with TOP at OCR0A
    TCCR2A |= (1 << WGM21);
    //enable the interrupt
    TIMSK2 = (1 << OCIE2A);
}

volatile int sonar_time;
volatile double sonar_distance = 500000;
volatile char sonar_wait;

ISR(TIMER2_COMPA_vect) //one interrupt per milisecond
{
    if (PINB & (1 << ECHO_PIN))
    {
        sonar_time += 500;
    }
}

ISR(PCINT1_vect)
{

    //daca trece pe 1 incepe o masuratoare
    if (PINB & (1 << ECHO_PIN))
    {
        sonar_time = -TCNT2 << 1;
        //prescaler 32
        //TCCR2B = ( 1 << CS21) | ( 1 << CS20);
    }
    else
    {                             //altfel o termina
        sonar_time += TCNT2 << 1; //16Mhz , prescaler 32 => 250kHz => 10^6/25*10^4 = 4 us per tick
        //TCCR2B = 0;
        sonar_wait = 0;
    }
}

volatile uint8_t sw_mpu_read_trigger;
volatile uint8_t sw_sonar_activation;
volatile uint16_t sw_buzzer_activation;
volatile uint8_t sw_stepper_activation;
/*
 * 0 - idle
 * 1 - read data from twi
 */
volatile uint8_t mpu_state;
volatile uint8_t sonar_activated;
volatile uint16_t adc_value;
volatile uint8_t check_connection;
volatile uint8_t update_servos;
volatile uint8_t update_pump;
volatile uint8_t stepper_activated;

/*
ADC
voltage / motor temperature
*/

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
    ADMUX &= ~( 1 << MUX0);
    ADCSRA |= (1 << ADSC);
}

void read_temp_sensor()
{
    ADMUX |= ( 1 << MUX0);
    ADCSRA |= (1 << ADSC);
}

void init_adc()
{
    DDRA &= ~(1 << PA0);
    DDRA &= ~(1 << PA1);

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

volatile uint8_t use_motors=0;
float motor_temperature; 
uint32_t motor_temp_adc;


ISR(ADC_vect)
{
    if( !use_motors)
    {
        adc_value = ADC;
        read_temp_sensor();
        use_motors = 1;
    } 
    else {
        motor_temp_adc = ADC;
        use_motors = 0;
    }
}

//pump trigger
uint8_t pump_engaged = 0;
uint8_t trigger_buzzer = 0;
uint8_t trigger_buzzer_counter = 0;

ISR(TIMER0_COMPA_vect)
{
    if(pump_engaged)
        PORTC |= (1 << PUMP_PWM_PIN);

    sw_mpu_read_trigger++;
    sw_sonar_activation++;
    sw_buzzer_activation++;
    sw_stepper_activation++;

    if (sw_mpu_read_trigger == 20)
    {
        mpu_state = 1;
        check_connection = 1;
        update_servos = 1;
        update_pump = 1;
        sw_mpu_read_trigger = 0;
    }

    if(sw_sonar_activation == 100) {
        sonar_activated = 1;
        sw_sonar_activation = 0;
    }

    if(sw_stepper_activation == 5) {
        stepper_activated = 1;
        sw_stepper_activation = 0;
    }

    if(sw_buzzer_activation == 1){
        if(trigger_buzzer || trigger_buzzer_counter>0){
            // PORT_TEST ^= (1 << PIN_SPK);
            PORT_TEST |= (1 << PIN_SPK);
        } else {
            PORT_TEST &= ~(1 << PIN_SPK);
        }  
    }
    if(sw_buzzer_activation == 100){
        if(trigger_buzzer || trigger_buzzer_counter>0){
            PORT_TEST &= ~(1 << PIN_SPK);
        }  
    }
    if(sw_buzzer_activation == 500) {
        sw_buzzer_activation = 0;
        if(trigger_buzzer_counter>0){
            trigger_buzzer_counter--;
        }
    }
}

ISR(TIMER0_COMPB_vect)
{
    if(pump_engaged)
        PORTC &= ~(1 << PUMP_PWM_PIN);
}


/*
Stepper motor stuff
*/

volatile uint16_t stepper_counter = 0;
volatile uint8_t start_stepper_motor = 0;

#define STEPPER_ZERO_TRIGGERED 0
#define STEPPER_WORKING 2
#define STEPPER_HUNDRED_TRGGERED 1
volatile uint8_t stepper_state = STEPPER_WORKING;
uint16_t stepper_target = 0;
uint32_t stepper_max_value;

ISR(STEPPER_PCINT)
{
    // logic low trigger
    if( (STEPPER_PIN & ( 1 << STEPPER_0_PIN)) == 0 )
    {
        if(stepper_state != STEPPER_ZERO_TRIGGERED){
            // opreste doar daca nu s-a oprit deja (permite apoi miscarea in sens opus)
            start_stepper_motor = 0;
            stepper_state = STEPPER_ZERO_TRIGGERED;
        }       
    }
    if( (STEPPER_PIN & ( 1 << STEPPER_100_PIN)) == 0 )
    {
        if(stepper_state != STEPPER_HUNDRED_TRGGERED){
            // opreste doar daca nu s-a oprit deja (permite apoi miscarea in sens opus)
            start_stepper_motor = 0;
            stepper_state = STEPPER_HUNDRED_TRGGERED;
        }  
    } 
}

void stepper_calibrate()
{
    USART0_print("9000,starting stepper calibration\r\n");
    start_stepper_motor = 1;
    stepper_max_value = 0;
    while(start_stepper_motor)
    {
        wdt_reset();
        if(stepper_activated)
        {
            full_step();
            stepper_activated = 0;
        }
    }
    stepper_direction = 0;
    start_stepper_motor = 1;

    while(start_stepper_motor)
    {
        wdt_reset();
        if(stepper_activated) {
            full_step();
            stepper_max_value++;
        }
    }

    sprintf(msg, "9000, stepper_max_value is: %ld\r\n", stepper_max_value);
    USART0_print(msg);
    update_setting(STEPPER_MAX_VALUE, stepper_max_value);
    save_setting();
}

void stepper_return_0(uint8_t blocking)
{
    if( (STEPPER_PIN & ( 1 << STEPPER_0_PIN)) == 0)
        return;

    stepper_direction = 0;
    start_stepper_motor = 1;

    if(blocking)
    {
        while(start_stepper_motor)
        {
            wdt_reset();
            if(stepper_activated) {
                full_step();
                stepper_max_value++;
            }
        }
    }
}

#define ESC_STOP 1000
#define ESC_START 1150

/*
 * 0 - aripi
 * 1 - carma
 */
void initialize_servos()
{
    servo_init_setup();
    DDRD |= (1 << PD6);
    DDRD |= (1 << PD7);
    DDRC |= (1 <<PC4);
    attach_servo(&PORTD, PD6); //ze flaps
    attach_servo(&PORTD, PD7); //ze carma
    attach_servo(&PORTC, PC4); //ze elice
    servo_init_ctrl();

    uint32_t carma_mid = get_settings_value_int(CARMA_POS);
    uint32_t wing_flaps_mid = get_settings_value_int(WING_FLAPS_POS);
    
    servo_set_cmd(0, carma_mid);
    servo_set_cmd(1, wing_flaps_mid);
    servo_set_cmd(2, ESC_STOP);

    poz_servos[0] = raw_servos[0] = carma_mid;
    poz_servos[1] = raw_servos[1] = wing_flaps_mid;
    poz_motors[0] = raw_motors[0] = ESC_STOP;
  
}

void setup()
{
    cli();
    // you can also DISABLE JTAG VIA FUSE to enable PC4, PC5

    MCUCR |= (1 << JTD); // Disable JTAG
    MCUCR |= (1 << JTD); // Disable JTAG

    MCUSR = 0;

    // [AP] added watchdog timer (useful for self-reset command, also prevent code from freezing, e.g. rogue while loop, blocking functions, etc)
    wdt_enable(WDTO_4S);

    gpio_init();

    USART0_init(115200);

    USART0_print("9000,hello world\r\n");

    sei();

    for (uint8_t i = 0; i < 2; i++)
    {
        PORT_TEST |= (1 << PIN_TEST);
        //PORT_TEST |= (1 << PIN_SPK);
        _delay_ms(500);
        PORT_TEST &= ~(1 << PIN_TEST);
        //PORT_TEST &= ~(1 << PIN_SPK);
        _delay_ms(500);
    }
    PORT_TEST |= (1 << PIN_TEST);

    wire_begin();
    mpu9250_v2_init();

    initialize_settings();
    
    init_mpu_timer();
    init_sonar_timer();

    init_ads1115();
    init_adc();
    initialize_servos();

    // timekeeper
    setup_timer();

    //setup stepper
    stepper_return_0(true);
    init_stepper();

    //stepper_motor_calibration
    stepper_calibrate();

    opperation_mode = AWAITING_START;

    beep_n(2);
}


void send_status()
{
    char mesaj[64];
    mesaj[0] = 0;

    append_command(mesaj, OUT_STATUS);
    append_int(mesaj, set_sail_engaged);

    append_int(mesaj,0);
    append_int(mesaj,0);
    append_int(mesaj,0);
    append_int(mesaj,0);
    append_int(mesaj,0);
    append_int(mesaj,0);

    append_int(mesaj,bench);
    append_int(mesaj,bench_ts);
    append_int(mesaj,return_home_engaged);

    append_int(mesaj,0);
    append_int(mesaj,0);
    append_int(mesaj,0);
    
    append_int(mesaj,pump_engaged);
    append_int(mesaj,0);
    strcat(mesaj,"\n");
    USART0_print(mesaj);
}

void send_sensors_data()
{
    char mesaj[64];
    mesaj[0] = 0;

    append_command(mesaj, OUT_SENSOR_DATA);

    append_int(mesaj, (int)pitch);                       //1
    append_int(mesaj, (int)roll);                      //2
    append_float(mesaj, 0);                             //3
    append_int(mesaj, (int)yaw);                        //4
    append_float(mesaj, 0);                             //5
    append_float(mesaj, 0);                             //6
    USART0_print(mesaj);        
    mesaj[0] = 0;
    append_float(mesaj, (float)stepper_counter/stepper_max_value);   //7
    // append_float(mesaj, current);
    append_float(mesaj, current);                       //8
    append_float(mesaj, 0);                             //9
    append_float(mesaj, 0);                             //10
    append_float(mesaj, 0);                             //11
    append_float(mesaj, starting_direction);            //12    
    append_float(mesaj, sonar_distance);                //13
    append_float(mesaj, bat1);                          //14
    append_float(mesaj, motor_temperature);                             //15
    strcat(mesaj, "\n");
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
        append_float(mesaj,  ( (float)(poz_motors[i]) - 1000) * 100.0 / (2000 - 1000));

    for (i = 0; i < N_SERVOS; i++)
        append_float(mesaj, ( (float)(poz_servos[i]) - 1000) * 100.0 / (2000 - 1000));

    strcat(mesaj, "\n");
    USART0_print(mesaj);
}


inline int limit_servo(int x, int upper, int lower)
{
    if (x > upper)
        return upper;
    if (x < lower)
        return lower;
    return x;
}

// [AP] poti folosi distanta de la sonar ca sa se opreasca la mal (daca "obstacolul" dispare, continua deplasarea), cu un threshold setat in config
//function that return the submarine home
// [VL] Done
void return_home()
{
    //compute the angle of ze carma so that the boat will return home
    if(sonar_distance < get_settings_value_float(RETURN_HOME_DISTANCE_POS)) {
        raw_motors[0] = poz_motors[0] = ESC_STOP;
        servo_set_cmd(2, poz_motors[0]);
    }
    else
    {
        raw_motors[0] = poz_motors[0] = get_settings_value_int(MOTOR_RETURN_POWER_POS);
        servo_set_cmd(2, poz_motors[0]);
        uint32_t carma_mid = get_settings_value_int(CARMA_POS);
        uint32_t up_carma = update_carma(&return_home_context, yaw) * 5 + carma_mid;
        raw_servos[0] = limit_servo(up_carma, carma_mid + 300, carma_mid - 300);
    }
}

void return_home_initialization()
{
    // use last heading as reference (keep going straight ahead)
    starting_direction = yaw;
    initialize_pid_contex(&return_home_context, dt, starting_direction);
    load_weights(&return_home_context, get_settings_value_float(KP_POS), get_settings_value_float(KI_POS),
                    get_settings_value_float(KD_POS));
    raw_motors[0] = poz_motors[0] = get_settings_value_int(MOTOR_RETURN_POWER_POS);
    raw_servos[0] = get_settings_value_int(CARMA_POS);
    raw_servos[1] = get_settings_value_int(WING_FLAPS_POS);
    servo_set_cmd(2, poz_motors[0]); //set a low speed for returning home
}

void beep()
{
    // PORT_TEST |= (1 << PIN_SPK);
    // _delay_ms(10);
    // PORT_TEST &= ~(1 << PIN_SPK);
    sw_buzzer_activation = 0;
    trigger_buzzer_counter = 1;
}

void beep_n(uint8_t n){
    sw_buzzer_activation = 0;
    trigger_buzzer_counter = n;
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

    uint32_t wing_flaps_mid = get_settings_value_int(WING_FLAPS_POS);
    uint32_t carma_mid = get_settings_value_int(CARMA_POS);

    switch (cmd)
    {
    case 1:
        // action 1
        // power motors
        // motor1 = data[1]; motor2 = data[2]
        // data[0] is cmd
        break;
    case WING_FLAPS:
        raw_servos[1] = limit_servo(data[1] * 5 + wing_flaps_mid, wing_flaps_mid + 300, wing_flaps_mid - 300);
        break;
    case CMD_POWER:
        raw_motors[0] = (data[1] < 0 ? 0 : data[1]) * 8 + ESC_START;
        stepper_target = ( (float)(data[2]+50) ) * get_settings_value_int(STEPPER_MAX_VALUE) / 100;
        if( stepper_counter > stepper_target){
            stepper_direction = 0;
            start_stepper_motor = 1;
        }
        else if( stepper_counter < stepper_target){
            stepper_direction = 1;
            start_stepper_motor = 1;
        }    
        break;
    case CARMA:
        raw_servos[0] = limit_servo(data[2] * 5 + carma_mid, carma_mid+300, carma_mid - 300);
        break;
    case CMD_SAVE_SETTINGS:
        save_setting();
        beep();
        break;
    case CMD_UPDATE_SETTINGS:
        update_setting(data[1], data[2]);
        beep();        
        break;
    case CMD_RESET_DEFAULTS:
        initialize_default();
        beep();
        break;
    case 202:
        send_sensors_data();
        break;
    case 203:
        send_motors_data();
        break;
    case 201:
        send_status();
        break;
    case CMD_REQUEST_SETTINGS:
        show_list();
        break;
    case SET_SAIL:
        opperation_mode = NORMAL_MODE;
        poz_motors[0] = ESC_START;
        raw_motors[0] = ESC_START;
        servo_set_cmd(2, poz_motors[0]);
        set_sail_engaged = 1;
        starting_direction = yaw - 180;
        if(starting_direction < 0)
            starting_direction += 360;
        beep();
        break;
    case HARBOUR:
        opperation_mode = AWAITING_START;
        poz_motors[0] = ESC_STOP;
        raw_motors[0] = ESC_STOP;
        servo_set_cmd(2, poz_motors[0]);
        set_sail_engaged = 0;
        beep();
        break;
    case RETURN_HOME_BUTTON:
        USART0_print("9000, return home\r\n");
        opperation_mode = RETURN_HOME;
        return_home_engaged = 1;
        return_home_initialization();
        beep();
        msg_received = 0;
        break;
    case RETURN_CONTROL:
        USART0_print("9000, return control\r\n");
        opperation_mode = NORMAL_MODE;
        return_home_engaged = 0;
        raw_motors[0] = ESC_START;
        beep();
        break;

    case START_PUMP:
        pump_engaged = 1;
        beep();
        break;

    case STOP_PUMP:
        PORTC &= ~(1 << PUMP_PWM_PIN); //put pump pin to low to disengage pump
        pump_engaged = 0;
        beep();
        break;
    
    case CMD_RESET:
        reset = 1;
        beep();
        break;

    case CALIBRATE_PUMP:
        stepper_calibrate();
        break;
    }
}

void compute_sonar_distance()
{
    if (sonar_wait == 0)
        sonar_distance = 0.85f * sonar_distance + 0.15f * sonar_time * 0.01715;
    sonar_wait = 1;
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);
}

int nr;
void loop()
{

    if (!reset)
    {
        wdt_reset();
    }
    else
    {
        // do not reset the watchdog => the mcu will reset
    }

    check_com(&onparse);

    // check for incoming data via USART0
    if (calibrate_magnetometer)
    {
        // [AP] de adaugat "9000," inainte de mesajele de debug
        // [VL] done
        if (nr == 0)
            USART0_print("9000, calibrating magnetometer\r\n");

        if (mpu_state == 1)
        {
            mpu9250_readMagData();
            mpu9250_calibrate();
            nr++;

            if (nr % 50 == 0)
            {
                sprintf(msg, "9000, %d\r\n", nr / 50);
                USART0_print(msg);
            }
            if (nr == 1500)
            {
                USART0_print("9000, done calibration\r\n");
                mpu9250_print_calib();
                calibrate_magnetometer = 0;
                nr = 0;
            }

            mpu_state = 0;
        }
        return;
    }

    if (mpu_state == 1)
    {
        uint32_t t0 = micros();
        uint32_t t1 = millis();
        bench_ts = t1 - ts_start;
        ts_start = t1;
        check_adc_module();
        read_adc();
        current = 0.9 * current + (float)adc_value * 7.33 / 1024 - 3.67;
        motor_temperature = 0.9 * motor_temperature + (float)(motor_temp_adc * 500) / 1024;
        //current = 0.9 * current + 0.1 * ( (float)adc_value *5 / 1024);
        mpu9250_v2_read();
        mpu9250_readMagData();
        mpu9250_correct_errors();
        mpu9250_compute_angles(dt);

        roll = mp_roll;
        pitch = mp_pitch;
        yaw = mp_yaw;

        mpu_state = 0;
        if (nr < 200)
        {
            err_roll += roll;
            err_pitch += pitch;
            //err_yaw += yaw;
            nr++;
        }
        else
        {
            roll -= err_roll;
            pitch -= err_pitch;
            //yaw -= err_yaw;
        }

        if (nr == 200)
        {
            err_roll /= 200;
            err_pitch /= 200;
            //err_yaw /=200;
            nr++;
        }
        else
        {
            roll = roll - err_roll;
            pitch = pitch - err_pitch;
            //yaw = yaw - err_yaw;
        }

        bench = micros() - t0;
    }

    if (sonar_activated == 1)
    {
        //sprintf(msg,"%d %d %d %.3f\r\n",mpu9250_magX, mpu9250_magY, mpu9250_magZ, yaw);
        //USART0_print(msg);
        compute_sonar_distance();
        sonar_activated = 0;
    }

    //check the connection with the bluetooth module
    if (check_connection == 1 && opperation_mode != AWAITING_START)
    {
        check_connection = 0;

        if (msg_received == 1)
        {
            msg_received = 0;
            lost_connection_counter = 0;
            if( opperation_mode == RETURN_HOME && !return_home_engaged)
            {
                raw_motors[0] = poz_motors[0] = ESC_START;
                servo_set_cmd(2, ESC_START);
                opperation_mode = NORMAL_MODE;
            }
        }
        else if (opperation_mode != RETURN_HOME)
        {
            lost_connection_counter++;
        }

        //after 2 seconds with no signal from the bluetooth, turn off the motor
        if (lost_connection_counter == 100)
        {
            raw_motors[0] = poz_motors[0] = ESC_STOP;
            servo_set_cmd(2, poz_motors[0]);
        }

        //after 10 more seconds, start the return home procedure
        if(lost_connection_counter == 600)
        {
            lost_connection_counter++;
            opperation_mode = RETURN_HOME;
            stepper_return_0(false); //bring it to the surface
            return_home_initialization();
        }
    }

        // [AP] aici mai avem un pas intermediar, sa astepte 5 min (opreste motor, asteapta timer) inainte sa se intoarca
    //done above
    if (opperation_mode == RETURN_HOME) 
    {
        return_home();
        trigger_buzzer = 1;
    } else {
        trigger_buzzer = 0;
    }

    float alph;
    if( update_servos == 1) 
    {
        update_servos = 0;

        alph = get_settings_value_float(ALPHA_CARMA_SERVO);
        poz_servos[0] = (int)(alph * (float)poz_servos[0] + (1 - alph) * (float)raw_servos[0]);
        servo_set_cmd(0, poz_servos[0]);

        alph = get_settings_value_float(ALPHA_WINGS_SERVO);
        poz_servos[1] = (int)(alph * (float)poz_servos[1] + (1 - alph) * (float)raw_servos[1]);
        
        servo_set_cmd(1, poz_servos[1]);

        if(opperation_mode != AWAITING_START)
        {
            alph = get_settings_value_float(ALPHA_ESC);
            if( motor_temperature > get_settings_value_float(MOTOR_MAX_TEMP))
                raw_motors[0] = ESC_START;
            poz_motors[0] = alph * poz_motors[0] + (1 - alph) * raw_motors[0];
            servo_set_cmd(2, poz_motors[0]);
        }
    }

    if(update_pump == 1)
    {
        update_pump = 0;
        alph = get_settings_value_float(ALPHA_PUMP_SOFT_START);
        OCR0B = alph * OCR0B + (1 - alph) * ((uint32_t)OCR0A) * get_settings_value_int(PUMP_DUTY_CYCLE) / 100;
    }

    
    if(stepper_activated == 1)
    {
        //a stepper step has been triggered( at 200HZ)
        stepper_activated = 0;

        //if the target is reached, stop the process
        if( ( stepper_direction == 1 && stepper_counter >= stepper_target) ||
            ( stepper_direction == 0 && stepper_counter <= stepper_target) )
        {
            start_stepper_motor = 0;
            //stepper_state = stepper_direction;
        }

        if( start_stepper_motor) //if the target has not been reached, do a step
        {
            stepper_state = STEPPER_WORKING;
            stepper_counter += stepper_direction == 1 ? 1 : -1;
            full_step();
        } 
        // else if( stepper_state != stepper_direction) {  //if the direction has been changed, do a step 
        //     start_stepper_motor = 1;                    //and start moving
        //     stepper_counter += stepper_direction == 1 ? 1 : -1;
        //     full_step();
        // } 
    }

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