
#include <math.h>
#include "mpu9250_v2.h"

float mpu9250_ax, mpu9250_ay, mpu9250_az;
float mpu9250_gx, mpu9250_gy, mpu9250_gz;
int16_t mpu9250_rawAccX, mpu9250_rawAccY, mpu9250_rawAccZ, mpu9250_rawTemp, mpu9250_rawGyroX, mpu9250_rawGyroY, mpu9250_rawGyroZ;
float mpu9250_temperature;
uint8_t mpu9250_accel_range, mpu9250_gyro_range;
float mp_roll, mp_pitch, mp_yaw;

uint8_t Mscale = MFS_16BITS;                                 // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;                                        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}; // Factory mag calibration and mag bias
float magBias[3] = {0, 0, 0}, magScale[3] = {0, 0, 0};

int16_t mpu9250_magX = 0, mpu9250_magY = 0, mpu9250_magZ = 0;

// https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu9250-accelerometer-and-gyroscope-tutorial/

uint8_t mpu9250_v2_init(void)
{
    uint8_t id = 0;

    wire_read_reg_byte(MPU9250_ADDRESS, MPU9250_WHO_AM_I, &id);

    MPU9250_DEBUGX2("MPU9250 read: ", MPU9250_WHO_AM_I, id);

    // make sure we're talking to the right chip
    if (id != MPU9250_DEVICE_ID)
    {
        return 0;
    }

    mpu9250_config();

// https://longnight975551865.wordpress.com/2018/02/11/how-to-read-data-from-mpu9250/
#ifdef MPU9250_ENABLE_MAG

    uint8_t BypassTrue = 0;

    while (BypassTrue == 0)
    {
        wire_write_reg_byte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
        wire_write_reg_byte(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
        uint8_t PinCFG;
        wire_read_reg_byte(MPU9250_ADDRESS, INT_PIN_CFG, &PinCFG); //0x22);
        uint8_t MasterDis;
        wire_read_reg_byte(MPU9250_ADDRESS, I2C_MST_CTRL, &MasterDis); //0x00); // Disable I2C master
        uint8_t IntEna;
        wire_read_reg_byte(MPU9250_ADDRESS, INT_ENABLE, &IntEna); //0x01); // Disable I2C master
        id = 0;
        wire_read_reg_byte(MPU9250_ADDRESS, WHO_AM_I_AK8963, &id); // Read WHO_AM_I register for MPU-9250

        // char msg[200];
        // sprintf(msg, "PinCFG: %x = 0x22 MasterDisable %x = 0x00 Interrupts %x = 0x01 Whoami %x = 0x71\n\r", PinCFG, MasterDis, IntEna, id);
        // USART0_print((const char *)msg);

        if (PinCFG == 0x22 && MasterDis == 0x00 && IntEna == 0x01)
        {
            BypassTrue = 1;
        }
        _delay_ms(800);
    }

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    wire_read_reg_byte(AK8963_ADDRESS, WHO_AM_I_AK8963, &id); // Read WHO_AM_I register for AK8963
    MPU9250_DEBUGX2("AK8963 read: ", WHO_AM_I_AK8963, id);

    if (id == AK8963_DEVICE_ID)
    {
        _delay_ms(100);

        // Get magnetometer calibration from AK8963 ROM
        mpu9250_initAK8963(magCalibration);
        MPU9250_DEBUG("AK8963 initialized for active data mode....\n\r", 0); // Initialize device for active mode read of magnetometer
    }
    else
    {
        return 0;
    }
#endif

    wire_read_reg_byte(MPU9250_ADDRESS, MPU9250_WHO_AM_I, &id);
    MPU9250_DEBUGX2("MPU9250 read: ", MPU9250_WHO_AM_I, id);

    return 1;
}

void mpu9250_read_bits(uint8_t reg, uint8_t *val, uint8_t _bits, uint8_t _shift)
{
    wire_read_reg_byte(MPU9250_ADDRESS, reg, val);
    *val >>= _shift;
    *val = (*val) & ((1 << (_bits)) - 1);
}

void mpu9250_write_bits(uint8_t reg, uint8_t data, uint8_t _bits, uint8_t _shift)
{
    uint8_t val = 0;
    mpu9250_read_bits(reg, &val, _bits, _shift);

    // mask off the data before writing
    uint8_t mask = (1 << (_bits)) - 1;
    data &= mask;

    mask <<= _shift;
    val &= ~mask;          // remove the current data at that spot
    val |= data << _shift; // and add in the new data

    wire_write_reg_byte(MPU9250_ADDRESS, reg, val);
}

uint8_t mpu9250_getSampleRateDivisor()
{
    uint8_t divisor = 0;
    wire_read_reg_byte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, &divisor);
    return divisor;
}

void mpu9250_setSampleRateDivisor(uint8_t divisor)
{
    wire_write_reg_byte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, divisor);
}

uint8_t mpu9250_getFilterBandwidth()
{
    uint8_t val = 0;
    mpu9250_read_bits(MPU9250_CONFIG, &val, 3, 0);
    return val;
}

void mpu9250_setFilterBandwidth(mpu9250_bandwidth_t val)
{
    mpu9250_write_bits(MPU9250_CONFIG, val, 3, 0);
}

uint8_t mpu9250_getGyroRange()
{
    uint8_t val = 0;
    mpu9250_read_bits(MPU9250_GYRO_CONFIG, &val, 2, 3);
    return val;
}

void mpu9250_setGyroRange(mpu9250_gyro_range_t val)
{
    mpu9250_write_bits(MPU9250_GYRO_CONFIG, val, 2, 3);
}

uint8_t mpu9250_getAccelerometerRange()
{
    uint8_t val = 0;
    mpu9250_read_bits(MPU9250_GYRO_CONFIG, &val, 2, 3);
    return val;
}

void mpu9250_setAccelerometerRange(mpu9250_accel_range_t val)
{
    mpu9250_write_bits(MPU9250_ACCEL_CONFIG, val, 2, 3);
}

void mpu9250_initAK8963(float *destination)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];                                     // x/y/z gyro calibration data stored here
    wire_write_reg_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    _delay_ms(10);
    wire_write_reg_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    _delay_ms(10);
    wire_read_reg_bytes(AK8963_ADDRESS, AK8963_ASAX, 3, rawData); // Read the x-, y-, and z-axis calibration values
    destination[0] = (float)(rawData[0] - 128) / 256. + 1.;       // Return x-axis sensitivity adjustment values, etc.
    destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
    destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
    wire_write_reg_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    _delay_ms(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    wire_write_reg_byte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    _delay_ms(10);
}

void mpu9250_config()
{

    mpu9250_setSampleRateDivisor(0);
    mpu9250_setFilterBandwidth(MPU9250_BAND_260_HZ);
    mpu9250_setGyroRange(MPU9250_RANGE_500_DEG);
    mpu9250_setAccelerometerRange(MPU9250_RANGE_2_G); // already the default

    uint8_t val = mpu9250_getSampleRateDivisor();
    MPU9250_DEBUGX("MPU9250 sample rate div: ", val);
    val = mpu9250_getFilterBandwidth();
    MPU9250_DEBUGX("MPU9250 filter bw: ", val);
    val = mpu9250_getGyroRange();
    mpu9250_gyro_range = val;

    MPU9250_DEBUGX("MPU9250 gyro range: ", val);
    val = mpu9250_getAccelerometerRange();
    mpu9250_accel_range = val;

    MPU9250_DEBUGX("MPU9250 acc range: ", val);

    mpu9250_reset();

    wire_write_reg_byte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x10);  //Set the register bits as 00010000 (1000dps full scale)
    wire_write_reg_byte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x10); //Set the register bits as 00010000 (+/- 8g full scale range)

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    wire_write_reg_byte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    wire_write_reg_byte(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt

    mpu9250_set();
}

void mpu9250_reset()
{
    // reset (place a 0 into the 6B register)
    wire_write_reg_byte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);
}

void mpu9250_set()
{
    // set (place a 1 into the 6B register)
    // set clock config to PLL with Gyro X reference
    wire_write_reg_byte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
}

void mpu9250_v2_read()
{

    // get raw readings
    uint8_t buffer[14];
    wire_read_reg_bytes(MPU9250_ADDRESS, MPU9250_ACCEL_OUT, 14, buffer);

    mpu9250_rawAccX = (uint16_t)buffer[0] << 8 | buffer[1];
    mpu9250_rawAccY = (uint16_t)buffer[2] << 8 | buffer[3];
    mpu9250_rawAccZ = (uint16_t)buffer[4] << 8 | buffer[5];

    mpu9250_rawTemp = (uint16_t)buffer[6] << 8 | buffer[7];

    mpu9250_rawGyroX = (uint16_t)buffer[8] << 8 | buffer[9];
    mpu9250_rawGyroY = (uint16_t)buffer[10] << 8 | buffer[11];
    mpu9250_rawGyroZ = (uint16_t)buffer[12] << 8 | buffer[13];

    // MPU9250_DEBUG("MPU9250 raw mpu9250_ax: ", mpu9250_rawAccX);
    // MPU9250_DEBUG("MPU9250 raw mpu9250_ay: ", mpu9250_rawAccY);
    // MPU9250_DEBUG("MPU9250 raw mpu9250_az: ", mpu9250_rawAccZ);

    // MPU9250_DEBUG("MPU9250 raw temp: ", mpu9250_rawTemp);

    // MPU9250_DEBUG("MPU9250 raw mpu9250_gx: ", mpu9250_rawGyroX);
    // MPU9250_DEBUG("MPU9250 raw mpu9250_gy: ", mpu9250_rawGyroY);
    // MPU9250_DEBUG("MPU9250 raw mpu9250_gz: ", mpu9250_rawGyroZ);

    mpu9250_temperature = (mpu9250_rawTemp / 340.0) + 36.53;

    // uint8_t mpu9250_accel_range = mpu9250_getAccelerometerRange();

    float accel_scale = 1;
    if (mpu9250_accel_range == MPU9250_RANGE_16_G)
        accel_scale = 2048;
    if (mpu9250_accel_range == MPU9250_RANGE_8_G)
        accel_scale = 4096;
    if (mpu9250_accel_range == MPU9250_RANGE_4_G)
        accel_scale = 8192;
    if (mpu9250_accel_range == MPU9250_RANGE_2_G)
        accel_scale = 16384;

    // setup range dependant scaling
    mpu9250_ax = ((float)mpu9250_rawAccX) / accel_scale;
    mpu9250_ay = ((float)mpu9250_rawAccY) / accel_scale;
    mpu9250_az = ((float)mpu9250_rawAccZ) / accel_scale;

    // uint8_t mpu9250_gyro_range = mpu9250_getGyroRange();

    float gyro_scale = 1;
    if (mpu9250_gyro_range == MPU9250_RANGE_250_DEG)
        gyro_scale = 131;
    if (mpu9250_gyro_range == MPU9250_RANGE_500_DEG)
        gyro_scale = 65.5;
    if (mpu9250_gyro_range == MPU9250_RANGE_1000_DEG)
        gyro_scale = 32.8;
    if (mpu9250_gyro_range == MPU9250_RANGE_2000_DEG)
        gyro_scale = 16.4;

    mpu9250_gx = ((float)mpu9250_rawGyroX) / gyro_scale;
    mpu9250_gy = ((float)mpu9250_rawGyroY) / gyro_scale;
    mpu9250_gz = ((float)mpu9250_rawGyroZ) / gyro_scale;

    MPU9250_DEBUGXF("MPU9250 raw ax: ", mpu9250_ax);
    MPU9250_DEBUGXF("MPU9250 raw ay: ", mpu9250_ay);
    MPU9250_DEBUGXF("MPU9250 raw az: ", mpu9250_az);

    MPU9250_DEBUGXF("MPU9250 raw temp: ", mpu9250_temperature);

    MPU9250_DEBUGXF("MPU9250 raw gx: ", mpu9250_gx);
    MPU9250_DEBUGXF("MPU9250 raw gy: ", mpu9250_gy);
    MPU9250_DEBUGXF("MPU9250 raw gz: ", mpu9250_gz);
}

void mpu9250_readMagData()
{
#ifdef MPU9250_ENABLE_MAG
    uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    uint8_t check = 0;

#ifndef MAG_NOBLOCK
    while (true)
    {
        wire_read_reg_byte(AK8963_ADDRESS, AK8963_ST1, &check);
        if (check & 0x01)
        {
            break;
        }
    }
#endif

#ifdef MAG_NOBLOCK
    wire_read_reg_byte(AK8963_ADDRESS, AK8963_ST1, &check);
    if (check & 0x01)
    {
#endif
        //{ // wait for magnetometer data ready bit to be set
        wire_read_reg_bytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, rawData); // Read the six raw data and ST2 registers sequentially into data array

        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if (!(c & 0x08))
        {                                                           // Check if magnetic sensor overflow set, if not then report data
            mpu9250_magX = ((int16_t)rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
            mpu9250_magY = ((int16_t)rawData[3] << 8) | rawData[2]; // Data stored as little Endian
            mpu9250_magZ = ((int16_t)rawData[5] << 8) | rawData[4];
        }
        // }
#ifdef MAG_NOBLOCK
    }
#endif
#endif
}

int8_t mpu9250_v2_getConvData(double *axg, double *ayg, double *azg, double *gxds, double *gyds, double *gzds)
{
    // *axg = (double)(mpu9250_ax - MPU9250_AXOFFSET) / MPU9250_AXGAIN;
    // *ayg = (double)(mpu9250_ay - MPU9250_AYOFFSET) / MPU9250_AYGAIN;
    // *azg = (double)(mpu9250_az - MPU9250_AZOFFSET) / MPU9250_AZGAIN;
    // *gxds = (double)(mpu9250_gx - MPU9250_GXOFFSET) / MPU9250_GXGAIN;
    // *gyds = (double)(mpu9250_gy - MPU9250_GYOFFSET) / MPU9250_GYGAIN;
    // *gzds = (double)(mpu9250_gz - MPU9250_GZOFFSET) / MPU9250_GZGAIN;

    *axg = (double)mpu9250_ax;
    *ayg = (double)mpu9250_ay;
    *azg = (double)mpu9250_az;
    *gxds = (double)mpu9250_gx;
    *gyds = (double)mpu9250_gy;
    *gzds = (double)mpu9250_gz;

    return 1;
}

int8_t mpu9250_v2_getConvDataMag(double *mx, double *my, double *mz)
{
    *mx = (double)mpu9250_magX;
    *my = (double)mpu9250_magY;
    *mz = (double)mpu9250_magZ;
    return 0;
}

void mpu9250_read_errors() {
    double eax,eay,eaz,egx,egy,egz;
    int c=0;
    eax=eay=eaz=egx=egy=egz=0;

    while(c<200){
        mpu9250_v2_read();
       
        c++;

        eax+=mpu9250_ax;
        eay+=mpu9250_ay;
        eaz+=mpu9250_az;
        egx+=mpu9250_gx;
        egy+=mpu9250_gy;
        egz+=mpu9250_gz;

        _delay_ms(500);
    }

    eax/=200;
    eay/=200;
    eaz/=200;
    egx/=200;
    egy/=200;
    egz/=200;
}

float err_ax = 0.502, err_ay = -0.025, err_az = 0.055;
float err_gx = 2.213, err_gy = -0.279, err_gz = 0.077;
int err_mx = 81, err_my = 267, err_mz = -124;

float gx_cor,gy_cor,gz_cor;
int magX_cor, magY_cor, magZ_cor;

void mpu9250_correct_errors(){
    // ax -= err_ax;
    // ay -= err_ay;
    // az -= err_az;
    gx_cor = mpu9250_gx - err_gx;
    gy_cor = mpu9250_gy - err_gy;
    gz_cor = mpu9250_gz - err_gz;
    magX_cor = mpu9250_magX - err_mx;
    magY_cor = mpu9250_magY - err_my;
    magZ_cor = mpu9250_magZ - err_mz;
}

#define SUB_DELTA 0.001
float angle_alpha = 0.96;
float gyro_factor = 1;
float dt=0.02;

void mpu9250_compute_angles()
{
    float ac_pitch, ac_roll, ac_yaw, den;

    den = sqrt( mpu9250_ay * mpu9250_ay + mpu9250_ax * mpu9250_ax);
    if (den < SUB_DELTA) den = SUB_DELTA;
    ac_pitch = -atan2(mpu9250_az,den);

    den = sqrt( mpu9250_az * mpu9250_az + mpu9250_ax * mpu9250_ax);
    if (den < SUB_DELTA) den = SUB_DELTA;
    ac_roll = atan2(mpu9250_ay,den);

    ac_yaw = atan2((float)magZ_cor, (float)magX_cor);
    if (ac_yaw < 0)
    {
        ac_yaw += 2 * M_PI;
    }
    if (ac_yaw > 2 * M_PI)
    {
        ac_yaw -= 2 * M_PI;
    }
    mp_yaw = ac_yaw *GRAD; //(mp_yaw + mpu9250_gx * gyro_factor * dt);   

    mp_pitch = (mp_pitch + -gy_cor * gyro_factor * dt) * angle_alpha + ac_pitch * GRAD * (1 - angle_alpha);
    mp_roll = (mp_roll + -gz_cor * gyro_factor * dt) * angle_alpha + ac_roll * GRAD * (1 - angle_alpha);
    
}

int ex,ey,ez;
int expl=-0x7fff, exm=0x7fff;
int eypl=-0x7fff, eym=0x7fff;
int ezpl=-0x7fff, ezm=0x7fff;

void mpu9250_calibrate()
{
    if( expl < mpu9250_magX) expl = mpu9250_magX;
    if( exm > mpu9250_magX) exm = mpu9250_magX;
    if( eypl < mpu9250_magY) eypl = mpu9250_magY;
    if( eym > mpu9250_magY) eym = mpu9250_magY;
    if( ezpl < mpu9250_magZ) ezpl = mpu9250_magZ;
    if( ezm > mpu9250_magZ) ezm = mpu9250_magZ;

    // ex+=mpu9250_magX;
    // ey+=mpu9250_magY;
    // ez+=mpu9250_magZ;
}

void mpu9250_print_calib()
{
    // err_mx = ex / 1000;
    // err_my = ey / 1000;
    // err_mz = ez / 1000;
    err_mx = (expl+exm)/2;
    err_my = (eypl+eym)/2;
    err_mz = (ezpl+ezm)/2;
    char msg[50];
    sprintf(msg,"%d %d %d\r\n",err_mx,err_my,err_mz);
    USART0_print(msg);
}
