

#ifndef MPU9250_V2_H_
#define MPU9250_V2_H_

// #define USE_TWI

#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "serial.h"
#include "serial_utils.h"

#include "twi_arduino.h"
#include "twi_utils.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MPU9250_ENABLE_MAG

// #define MPU9250_DEBUG_ENABLE

#define MAG_NOBLOCK

#ifndef MPU9250_DEBUG

#ifdef MPU9250_DEBUG_ENABLE
#define MPU9250_DEBUG(name, val) serial0_print_kv((char *)name, val, 1)
#define MPU9250_DEBUGX(name, val) serial0_print_kvx((char *)name, val, "%ld", 1)
#define MPU9250_DEBUGXF(name, val) serial0_print_kvx((char *)name, (int32_t)(val * 100), "%ld", 1)
#define MPU9250_DEBUGX2(name, reg, val)            \
    serial0_print_kvx((char *)name, reg, "%x", 0); \
    serial0_print_kvx((char *)" val: ", val, "%d", 1)
#endif

#ifndef MPU9250_DEBUG_ENABLE

#define MPU9250_DEBUG(name, val) \
    do                           \
    {                            \
    } while (0)
#define MPU9250_DEBUGX(name, val) \
    do                            \
    {                             \
    } while (0)
#define MPU9250_DEBUGXF(name, val) \
    do                             \
    {                              \
    } while (0)
#define MPU9250_DEBUGX2(name, reg, val) \
    do                                  \
    {                                   \
    } while (0)
#endif
#endif

#define MPU9250_ADDRESS \
    0x68                       ///< MPU9250 default i2c address w/ AD0 high

// #define MPU9250_DEVICE_ID 0x68 ///< The correct MPU6050_WHO_AM_I value
#define MPU9250_DEVICE_ID 0x71 ///< The correct MPU9250_WHO_AM_I value

#define AK8963_DEVICE_ID 0x48

#define MPU9250_SELF_TEST_X \
    0x0D ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Y \
    0x0E ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Z \
    0x0F ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_A \
    0x10                         ///< Self test factory calibrated values register
#define MPU9250_SMPLRT_DIV 0x19  ///< sample rate divisor register
#define MPU9250_CONFIG 0x1A      ///< General configuration register
#define MPU9250_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU9250_ACCEL_CONFIG \
    0x1C                               ///< Accelerometer specific configration register
#define MPU9250_INT_PIN_CONFIG 0x37    ///< Interrupt pin configuration register
#define MPU9250_WHO_AM_I 0x75          ///< Divice ID register
#define MPU9250_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU9250_USER_CTRL 0x6A         ///< FIFO and I2C Master control register
#define MPU9250_PWR_MGMT_1 0x6B        ///< Primary power/sleep control register
#define MPU9250_PWR_MGMT_2 0x6C        ///< Secondary power/sleep control register
#define MPU9250_TEMP_H 0x41            ///< Temperature data high byte register
#define MPU9250_TEMP_L 0x42            ///< Temperature data low byte register
#define MPU9250_ACCEL_OUT 0x3B         ///< base address for sensor data reads


#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define I2C_MST_CTRL     0x24  

#define MPU9250_CALIBRATEDACCGYRO 1 //set to 1 if is calibrated
#if MPU9250_CALIBRATEDACCGYRO == 1
#define MPU9250_AXOFFSET 0
#define MPU9250_AYOFFSET 0
#define MPU9250_AZOFFSET 0
#define MPU9250_AXGAIN 16384.0
#define MPU9250_AYGAIN 16384.0
#define MPU9250_AZGAIN 16384.0
#define MPU9250_GXOFFSET -42
#define MPU9250_GYOFFSET 9
#define MPU9250_GZOFFSET -29
#define MPU9250_GXGAIN 16.4
#define MPU9250_GYGAIN 16.4
#define MPU9250_GZGAIN 16.4
#endif

//Magnetometer Registers
#define AK8963_ADDRESS 0x0C
#define WHO_AM_I_AK8963 0x00 // should return 0x48
#define INFO 0x01
#define AK8963_ST1 0x02    // data ready status bit 0
#define AK8963_XOUT_L 0x03 // data
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09    // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL 0x0A   // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC 0x0C   // Self test control
#define AK8963_I2CDIS 0x0F // I2C disable
#define AK8963_ASAX 0x10   // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY 0x11   // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ 0x12   // Fuse ROM z-axis sensitivity adjustment value

    /**
 * @brief FSYNC output values
 *
 * Allowed values for `setFsyncSampleOutput`.
 */
    typedef enum fsync_out
    {
        MPU9250_FSYNC_OUT_DISABLED,
        MPU9250_FSYNC_OUT_TEMP,
        MPU9250_FSYNC_OUT_GYROX,
        MPU9250_FSYNC_OUT_GYROY,
        MPU9250_FSYNC_OUT_GYROZ,
        MPU9250_FSYNC_OUT_ACCELX,
        MPU9250_FSYNC_OUT_ACCELY,
        MPU9250_FSYNC_OUT_ACCEL_Z,
    } mpu9250_fsync_out_t;

    /**
 * @brief Clock source options
 *
 * Allowed values for `setClock`.
 */
    typedef enum clock_select
    {
        MPU9250_INTR_8MHz,
        MPU9250_PLL_GYROX,
        MPU9250_PLL_GYROY,
        MPU9250_PLL_GYROZ,
        MPU9250_PLL_EXT_32K,
        MPU9250_PLL_EXT_19MHz,
        MPU9250_STOP = 7,
    } mpu9250_clock_select_t;

    /**
 * @brief Accelerometer range options
 *
 * Allowed values for `mpu9250_setAccelerometerRange`.
 */
    typedef enum
    {
        MPU9250_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
        MPU9250_RANGE_4_G = 0b01,  ///< +/- 4g
        MPU9250_RANGE_8_G = 0b10,  ///< +/- 8g
        MPU9250_RANGE_16_G = 0b11, ///< +/- 16g
    } mpu9250_accel_range_t;

    /**
 * @brief Gyroscope range options
 *
 * Allowed values for `mpu9250_setGyroRange`.
 */
    typedef enum
    {
        MPU9250_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
        MPU9250_RANGE_500_DEG,  ///< +/- 500 deg/s
        MPU9250_RANGE_1000_DEG, ///< +/- 1000 deg/s
        MPU9250_RANGE_2000_DEG, ///< +/- 2000 deg/s
    } mpu9250_gyro_range_t;

    /**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `mpu9250_setFilterBandwidth`.
 */
    typedef enum
    {
        MPU9250_BAND_260_HZ, ///< Docs imply this disables the filter
        MPU9250_BAND_184_HZ, ///< 184 Hz
        MPU9250_BAND_94_HZ,  ///< 94 Hz
        MPU9250_BAND_44_HZ,  ///< 44 Hz
        MPU9250_BAND_21_HZ,  ///< 21 Hz
        MPU9250_BAND_10_HZ,  ///< 10 Hz
        MPU9250_BAND_5_HZ,   ///< 5 Hz
    } mpu9250_bandwidth_t;

    /**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
    typedef enum
    {
        MPU9250_CYCLE_1_25_HZ, ///< 1.25 Hz
        MPU9250_CYCLE_5_HZ,    ///< 5 Hz
        MPU9250_CYCLE_20_HZ,   ///< 20 Hz
        MPU9250_CYCLE_40_HZ,   ///< 40 Hz
    } mpu9250_cycle_rate_t;

    typedef enum
    {
        MFS_14BITS = 0, // 0.6 mG per LSB
        MFS_16BITS      // 0.15 mG per LSB
    } Mscale_values;

    extern float ax, ay, az;
    extern float gx, gy, gz;
    extern int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    extern float mp_roll, mp_pitch, mp_yaw;
    extern float temperature;

    extern int16_t mpu9250_magX , mpu9250_magY , mpu9250_magZ;
    extern uint8_t accel_range, gyro_range;

    //errors
    extern float err_ax , err_ay, err_az;
    extern float err_gx , err_gy, err_gz;
    extern int err_mx, err_my, err_mz;

    //functions
    void mpu9250_set();
    void mpu9250_reset();
    void mpu9250_config();
    uint8_t mpu9250_v2_init(void);
    void mpu9250_initAK8963(float *destination);

    void mpu9250_v2_read(void);
    void mpu9250_readMagData();

    int8_t mpu9250_v2_getConvData(double *axg, double *ayg, double *azg, double *gxds, double *gyds, double *gzds);
    int8_t mpu9250_v2_getConvDataMag(double *mx, double *my, double *mz);

    void mpu9250_read_bits(uint8_t reg, uint8_t *val, uint8_t _bits, uint8_t _shift);
    void mpu9250_write_bits(uint8_t reg, uint8_t data, uint8_t _bits, uint8_t _shift);

    uint8_t mpu9250_getSampleRateDivisor();

    void mpu9250_setSampleRateDivisor(uint8_t divisor);

    uint8_t mpu9250_getFilterBandwidth();

    void mpu9250_setFilterBandwidth(mpu9250_bandwidth_t val);

    uint8_t mpu9250_getGyroRange();

    void mpu9250_setGyroRange(mpu9250_gyro_range_t val);

    uint8_t mpu9250_getAccelerometerRange();

    void mpu9250_setAccelerometerRange(mpu9250_accel_range_t val);

    void mpu9250_correct_errors();
    void mpu9250_compute_angles(float dt);
    void mpu9250_calibrate();
    void mpu9250_print_calib();

#ifdef __cplusplus
}
#endif

#endif