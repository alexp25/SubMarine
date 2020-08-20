#include "eeprom_config.h"

#define DEBUG_EEPROM

#ifdef DEBUG_EEPROM
char msg_eeprom[100];
#endif

void mpu9250_write_a_errors(float eax,float eay,float eaz)
{
    #ifdef DEBUG_EEPROM
        USART0_print("writing a errors\r\n");
    #endif
    eeprom_write_dword((uint32_t*)1,eax);
    eeprom_write_dword((uint32_t*)5,eay);
    eeprom_write_dword((uint32_t*)9,eaz);
}

void mpu9250_write_g_errors(float egx, float egy, float egz)
{
    #ifdef DEBUG_EEPROM
        USART0_print("writing g errors\r\n");
    #endif
    eeprom_write_dword((uint32_t*)13,egx);
    eeprom_write_dword((uint32_t*)17,egy);
    eeprom_write_dword((uint32_t*)21,egz);
}

void mpu9250_write_m_errors(int emx, int emy, int emz)
{
    #ifdef DEBUG_EEPROM
        USART0_print("writing m errors\r\n");
    #endif
    eeprom_write_dword((uint32_t*)25,emx);
    eeprom_write_dword((uint32_t*)29,emy);
    eeprom_write_dword((uint32_t*)33,emz);
}

void mpu9250_initialize_errors() 
{
    unsigned char sentinel;

    sentinel = eeprom_read_byte(0);
    if( sentinel == EEPROM_SENTINEL) {
        #ifdef DEBUG_EEPROM
            USART0_print("mpu initialize errors from eeprom\r\n");
        #endif
        err_ax = ((float) eeprom_read_dword((const uint32_t*)1))/1000;
        err_ay = ((float) eeprom_read_dword((const uint32_t*)5))/1000;
        err_az = ((float) eeprom_read_dword((const uint32_t*)9))/1000;
        err_gx = ((float) eeprom_read_dword((const uint32_t*)13))/1000;
        err_gy = ((float) eeprom_read_dword((const uint32_t*)17))/1000;
        err_gz = ((float) eeprom_read_dword((const uint32_t*)21))/1000;
        err_mx = eeprom_read_dword((const uint32_t*)25);
        err_my = eeprom_read_dword((const uint32_t*)29);
        err_mz = eeprom_read_dword((const uint32_t*)33);
    }
    #ifdef DEBUG_EEPROM
        else USART0_print("mpu using defaults\r\n");
    #endif
    #ifdef DEBUG_EEPROM
        sprintf(msg_eeprom, "%.3f %.3f %.3f\r\n",err_ax, err_ay, err_az);
        USART0_print(msg_eeprom);
        sprintf(msg_eeprom, "%.3f %.3f %.3f\r\n",err_gx, err_gy, err_gz);
        USART0_print(msg_eeprom);
        sprintf(msg_eeprom, "%d %d %d\r\n",err_mx, err_my, err_mz);
        USART0_print(msg_eeprom);
    #endif
}

void write_sentinel(Sentinel_Device dev, uint8_t value)
{
    #ifdef DEBUG_EEPROM
        sprintf(msg_eeprom, "writing device %d sentinel with value %d", dev, value);
        USART0_print(msg_eeprom);
    #endif

    switch (dev)
    {
    case MPU:
        eeprom_write_byte((uint8_t*)0, value);
        break;
    
    case PID:
        eeprom_write_byte((uint8_t*)37, value);
        break;

    default:
        break;
    }
}


void pid_initialize_errors()
{
    unsigned char sentinel;

    sentinel = eeprom_read_byte((const uint8_t*)37);
    if( sentinel == EEPROM_SENTINEL) {
        kp = ((float)eeprom_read_dword((const uint32_t*)38))/10000;
        ki = ((float)eeprom_read_dword((const uint32_t*)42))/10000;
        kd = ((float)eeprom_read_dword((const uint32_t*)46))/10000;
        sprintf(msg_eeprom, "from eeprom %.3f %.3f %.3f\r\n",kp, ki, kd);
        USART0_print(msg_eeprom);
    } 
    #ifdef DEBUG_EEPROM
        else {
            sprintf(msg_eeprom, "use defaults %.3f %.3f %.3f\r\n",kp, ki, kd);
            USART0_print(msg_eeprom);
        }
    #endif
}

void pid_write_coefficients(float kp,float ki, float kd)
{
    #ifdef DEBUG_EEPROM
        sprintf(msg_eeprom, " writing pid coefficients %.3f %.3f %.3f\r\n",kp,ki,kd);
        USART0_print(msg_eeprom);
    #endif
    eeprom_write_dword((uint32_t*)38,kp);
    eeprom_write_dword((uint32_t*)42,ki);
    eeprom_write_dword((uint32_t*)46,kd);
}