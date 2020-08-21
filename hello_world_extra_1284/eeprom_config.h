#include "mpu9250_v2.h"
#include "pid.h"
#include <avr/eeprom.h>
#include "serial.h"
// [AP] e cam anti-pattern sa incluzi main aici
// de ex. pt citirea coef pid, functia pid_initialize_errors modifica variabilele din pid.c
#include "main.h"
#define EEPROM_SENTINEL 0xFA
#define EEPROM_DEFAULTS 0x00

enum Sentinel_Device{ 
    MPU = 0,
    PID = 1
};

void mpu9250_write_a_errors(float eax,float eay,float eaz);
void mpu9250_write_g_errors(float egx, float egy, float egz);
void mpu9250_write_m_errors(int emx, int emy, int emz);
void mpu9250_initialize_errors();

void write_sentinel(Sentinel_Device dev, uint8_t value);

void pid_initialize_errors();
void pid_write_coefficients(float kp,float ki, float kd);

void servo_initialize_bias(int dev);
void servo_write_bias(int dev, short value);