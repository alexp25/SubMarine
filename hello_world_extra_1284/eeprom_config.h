#include "mpu9250_v2.h"
#include "pid.h"
#include <avr/eeprom.h>
#include "serial.h"
#define EEPROM_SENTINEL 0xFA
#define EEPROM_DEFAULTS 0x00

enum Sentinel_Device{ MPU = 0, PID = 1};

void mpu9250_write_a_errors(float eax,float eay,float eaz);
void mpu9250_write_g_errors(float egx, float egy, float egz);
void mpu9250_write_m_errors(int emx, int emy, int emz);
void mpu9250_initialize_errors();

void write_sentinel(Sentinel_Device dev, uint8_t value);

void pid_initialize_errors();
void pid_write_coefficients(float kp,float ki, float kd);