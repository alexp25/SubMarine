#define CARMA 7
#define SINK_ANGLE 6
#define WING_FLAPS 2
#define MOTOR 8

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
#define CMD_RESET 111

#define OUT_MOTOR_DATA 1
#define OUT_ACK 200
#define OUT_NACK 400
#define OUT_STATUS 3
#define OUT_SENSOR_DATA 4
#define OUT_SETTINGS 5

extern int servo_carma_mid_position;
extern int servo_carma_upper_limit;
extern int servo_carma_lower_limit;

extern int servo_wings_mid_position;
extern int servo_wings_upper_limit;
extern int servo_wings_lower_limit;

extern int motor_return_power;
