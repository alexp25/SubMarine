#include <stdint.h>

#define SERVO_IDX_MOTOR 2
#define SERVO_IDX_FLAPS 1
#define SERVO_IDX_CARMA 0

#define CARMA 2
#define CMD_POWER 6
#define WING_FLAPS 7
#define MOTOR 8
#define SET_SAIL 4
#define HARBOUR 5
#define STEPPER_SLIDER_IDX 4

#define CALIBRATE_PUMP 101

#define RETURN_HOME_BUTTON 102
#define RETURN_CONTROL 103

#define START_PUMP 131
#define STOP_PUMP 132

#define READ_MPU 10
#define WRITE_MPU_A 11
#define WRITE_MPU_G 12
#define WRITE_MPU_M 13
#define READ_PID 14
#define WRITE_PID 15
#define WRITE_SENTINEL 16
#define CLEAR_SENTINEL 17
#define READ_SERVOS 18
#define WRITE_SERVOS 19
#define REWRITE_PID_PARAMETERS 20
#define WRITE_DISTANCE_TRESHOLD 21
#define READ_DISTANCE_TRESHOLD 22

#define CMD_UPDATE_SETTINGS 105
#define CMD_SAVE_SETTINGS 106
#define CMD_RESET_DEFAULTS 107
#define CMD_RESET 111
#define CMD_REQUEST_SETTINGS 206


#define OUT_MOTOR_DATA 1
#define OUT_ACK 200
#define OUT_NACK 400
#define OUT_STATUS 3
#define OUT_SENSOR_DATA 4
#define OUT_SETTINGS 5

#define N_MOTORS 1
#define N_SERVOS 2
int poz_servos[N_SERVOS];
int poz_motors[N_MOTORS];

int raw_servos[N_SERVOS];
int raw_motors[N_MOTORS];

void beep();
void beep_n(uint8_t n);