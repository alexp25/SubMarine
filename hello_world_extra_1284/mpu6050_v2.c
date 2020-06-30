
#include "mpu6050_v2.h"

float ax, ay, az;
float gx, gy, gz;
int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
float temperature;

uint8_t accel_range, gyro_range;

// https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

uint8_t mpu6050_v2_init(void)
{
    uint8_t id = 0;

    wire_read_reg_byte(MPU6050_I2CADDR_DEFAULT, MPU6050_WHO_AM_I, &id);

    // make sure we're talking to the right chip
    if (id != MPU6050_DEVICE_ID)
    {
        return 0;
    }

    mpu6050_config();

    return 1;
}

void mpu_read_bits(uint8_t reg, uint8_t *val, uint8_t _bits, uint8_t _shift)
{
    wire_read_reg_byte(MPU6050_I2CADDR_DEFAULT, reg, val);
    *val >>= _shift;
    *val = (*val) & ((1 << (_bits)) - 1);
}

void mpu_write_bits(uint8_t reg, uint8_t data, uint8_t _bits, uint8_t _shift)
{
    uint8_t val = 0;
    mpu_read_bits(reg, &val, _bits, _shift);

    // mask off the data before writing
    uint8_t mask = (1 << (_bits)) - 1;
    data &= mask;

    mask <<= _shift;
    val &= ~mask;          // remove the current data at that spot
    val |= data << _shift; // and add in the new data

    wire_write_reg_byte(MPU6050_I2CADDR_DEFAULT, reg, val);
}

uint8_t getSampleRateDivisor()
{
    uint8_t divisor = 0;
    wire_read_reg_byte(MPU6050_I2CADDR_DEFAULT, MPU6050_SMPLRT_DIV, &divisor);
    return divisor;
}

void setSampleRateDivisor(uint8_t divisor)
{
    wire_write_reg_byte(MPU6050_I2CADDR_DEFAULT, MPU6050_SMPLRT_DIV, divisor);
}

uint8_t getFilterBandwidth()
{
    uint8_t val = 0;
    mpu_read_bits(MPU6050_CONFIG, &val, 3, 0);
    return val;
}

void setFilterBandwidth(mpu6050_bandwidth_t val)
{
    mpu_write_bits(MPU6050_CONFIG, val, 3, 0);
}

uint8_t getGyroRange()
{
    uint8_t val = 0;
    mpu_read_bits(MPU6050_GYRO_CONFIG, &val, 2, 3);
    return val;
}

void setGyroRange(mpu6050_gyro_range_t val)
{
    mpu_write_bits(MPU6050_GYRO_CONFIG, val, 2, 3);
}

uint8_t getAccelerometerRange()
{
    uint8_t val = 0;
    mpu_read_bits(MPU6050_GYRO_CONFIG, &val, 2, 3);
    return val;
}

void setAccelerometerRange(mpu6050_accel_range_t val)
{
    mpu_write_bits(MPU6050_ACCEL_CONFIG, val, 2, 3);
}

void mpu6050_config()
{

    setSampleRateDivisor(0);
    setFilterBandwidth(MPU6050_BAND_260_HZ);
    setGyroRange(MPU6050_RANGE_500_DEG);
    setAccelerometerRange(MPU6050_RANGE_2_G); // already the default

    uint8_t val = getSampleRateDivisor();
    val = getFilterBandwidth();
    val = getGyroRange();
    gyro_range = val;
    val = getAccelerometerRange();
    accel_range = val;
    mpu6050_reset();
    wire_write_reg_byte(MPU6050_I2CADDR_DEFAULT, MPU6050_GYRO_CONFIG, 0x10);  //Set the register bits as 00010000 (1000dps full scale)
    wire_write_reg_byte(MPU6050_I2CADDR_DEFAULT, MPU6050_ACCEL_CONFIG, 0x10); //Set the register bits as 00010000 (+/- 8g full scale range)
    mpu6050_set();
}

void mpu6050_reset()
{
    // reset (place a 0 into the 6B register)
    wire_write_reg_byte(MPU6050_I2CADDR_DEFAULT, MPU6050_PWR_MGMT_1, 0x00);
}

void mpu6050_set()
{
    // set (place a 1 into the 6B register)
    // set clock config to PLL with Gyro X reference
    wire_write_reg_byte(MPU6050_I2CADDR_DEFAULT, MPU6050_PWR_MGMT_1, 0x01);
}

void mpu6050_v2_read()
{

    // get raw readings
    uint8_t buffer[14];
    wire_read_reg_bytes(MPU6050_I2CADDR_DEFAULT, MPU6050_ACCEL_OUT, 14, buffer);

    rawAccX = (uint16_t)buffer[0] << 8 | buffer[1];
    rawAccY = (uint16_t)buffer[2] << 8 | buffer[3];
    rawAccZ = (uint16_t)buffer[4] << 8 | buffer[5];

    rawTemp = (uint16_t)buffer[6] << 8 | buffer[7];

    rawGyroX = (uint16_t)buffer[8] << 8 | buffer[9];
    rawGyroY = (uint16_t)buffer[10] << 8 | buffer[11];
    rawGyroZ = (uint16_t)buffer[12] << 8 | buffer[13];

    temperature = (rawTemp / 340.0) + 36.53;

    // uint8_t accel_range = getAccelerometerRange();

    float accel_scale = 1;
    if (accel_range == MPU6050_RANGE_16_G)
        accel_scale = 2048;
    if (accel_range == MPU6050_RANGE_8_G)
        accel_scale = 4096;
    if (accel_range == MPU6050_RANGE_4_G)
        accel_scale = 8192;
    if (accel_range == MPU6050_RANGE_2_G)
        accel_scale = 16384;

    // setup range dependant scaling
    ax = ((float)rawAccX) / accel_scale;
    ay = ((float)rawAccY) / accel_scale;
    az = ((float)rawAccZ) / accel_scale;

    // uint8_t gyro_range = getGyroRange();

    float gyro_scale = 1;
    if (gyro_range == MPU6050_RANGE_250_DEG)
        gyro_scale = 131;
    if (gyro_range == MPU6050_RANGE_500_DEG)
        gyro_scale = 65.5;
    if (gyro_range == MPU6050_RANGE_1000_DEG)
        gyro_scale = 32.8;
    if (gyro_range == MPU6050_RANGE_2000_DEG)
        gyro_scale = 16.4;

    gx = ((float)rawGyroX) / gyro_scale;
    gy = ((float)rawGyroY) / gyro_scale;
    gz = ((float)rawGyroZ) / gyro_scale;
}

int8_t mpu6050_v2_getConvData(double *axg, double *ayg, double *azg, double *gxds, double *gyds, double *gzds)
{
    // *axg = (double)(ax - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
    // *ayg = (double)(ay - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
    // *azg = (double)(az - MPU6050_AZOFFSET) / MPU6050_AZGAIN;
    // *gxds = (double)(gx - MPU6050_GXOFFSET) / MPU6050_GXGAIN;
    // *gyds = (double)(gy - MPU6050_GYOFFSET) / MPU6050_GYGAIN;
    // *gzds = (double)(gz - MPU6050_GZOFFSET) / MPU6050_GZGAIN;

    *axg = (double)ax;
    *ayg = (double)ay;
    *azg = (double)az;
    *gxds = (double)gx;
    *gyds = (double)gy;
    *gzds = (double)gz;

    return 1;
}
