
#include "ads1115.h"

uint8_t m_i2cAddress;      ///< the I2C address
uint8_t m_conversionDelay; ///< conversion deay
uint8_t m_bitShift;        ///< bit shift amount
uint16_t m_gain;           ///< ADC gain
uint8_t sweep_chan = 0;
uint8_t flag_started_timeout[N_CHAN_ADS];
uint16_t adc_buffer[N_CHAN_ADS];

void init_ads1115()
{
    m_i2cAddress = ADS1115_ADDRESS;
    m_conversionDelay = ADS1115_CONVERSIONDELAY;
    m_bitShift = 0;
    m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
    for (uint8_t i = 0; i < N_CHAN_ADS; i++)
    {
        flag_started_timeout[i] = 0;
    }
}

uint16_t readADC_SingleEnded(uint8_t channel)
{
    if (!startADC_SingleEnded(channel))
    {
        return 0;
    }

    // Wait for the conversion to complete
    _delay_ms(m_conversionDelay);

    // Read the conversion results
    // Shift 12-bit results right 4 bits for the ADS1015
    uint16_t reg;
    wire_read_reg_16(m_i2cAddress, ADS1015_REG_POINTER_CONVERT, &reg);
    return reg >> m_bitShift;
}

uint8_t startADC_SingleEnded(uint8_t channel)
{
    if (channel > 3)
    {
        return 0;
    }

    // Start with default values
    uint16_t config =
        ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
        ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
        ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
        ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
        ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
        ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

    // Set PGA/voltage range
    config |= m_gain;

    // Set single-ended input channel
    switch (channel)
    {
    case (0):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
        break;
    case (1):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
        break;
    case (2):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
        break;
    case (3):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
        break;
    }

    // Set 'start single-conversion' bit
    config |= ADS1015_REG_CONFIG_OS_SINGLE;

    // Write config register to the ADC
    wire_write_reg_16(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
    return 1;
}

uint8_t check_ready()
{
    uint16_t config_reg;
    wire_read_reg_16(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, &config_reg);
    // Serial.println(config_reg, HEX);
    // Serial.println(config_reg & ADS1015_REG_CONFIG_OS_NOTBUSY, HEX);
    return (config_reg & ADS1015_REG_CONFIG_OS_NOTBUSY) != 0;
}

/* 
reads single channels one by one, returns ready when all channels were read 
min delay between calls: 1 ms (10ms conversion time/10 timeout)
*/
uint8_t sweep_read_noblock(uint16_t *adc_val)
{
    uint8_t readyall = 0;
    if (check_ready())
    {
        // conversion is ready, get adc value, double buffered
        wire_read_reg_16(m_i2cAddress, ADS1015_REG_POINTER_CONVERT, &(adc_buffer[sweep_chan]));
        adc_buffer[sweep_chan] = adc_buffer[sweep_chan] >> m_bitShift;
        // prepare next channel
        sweep_chan += 1;
        if (sweep_chan >= N_CHAN_ADS)
        {
            // start over
            sweep_chan = 0;
            readyall = 1;
            for (uint8_t i = 0; i < N_CHAN_ADS; i++)
            {
                flag_started_timeout[i] = 0;
                adc_val[i] = adc_buffer[i];
            }
        }
    }
    else
    {
        // not ready yet, timeout if not ready for N function calls
        if (flag_started_timeout[sweep_chan] > 0)
        {
            flag_started_timeout[sweep_chan] -= 1;
        }
    }

    // start conversion (if not already started)
    if (!flag_started_timeout[sweep_chan])
    {
        // reset timeout
        flag_started_timeout[sweep_chan] = 10;
        // start conversion
        startADC_SingleEnded(sweep_chan);
    }

    return readyall;
}

uint8_t read_noblock_channel(uint16_t *adc_val, uint8_t channel)
{
    uint8_t ready = 0;
    if (check_ready())
    {
        wire_read_reg_16(m_i2cAddress, ADS1015_REG_POINTER_CONVERT, adc_val);
        *adc_val = *adc_val >> m_bitShift;
        // if (*adc_val > 32767)
        // {
        //     *adc_val = 0;
        // }
        ready = 1;
    }
    else
    {
        ready = 0;
    }
    startADC_SingleEnded(channel);
    return ready;
}

double to_volts(uint16_t adc_val)
{
    double volts = 0;
    volts = adc_val * 6.144 / 32768;
    return volts;
}

uint16_t to_mv(uint16_t adc_val)
{
    uint16_t mv = 0;
    mv = (uint32_t)adc_val * 6144 / 32768;
    return mv;
}