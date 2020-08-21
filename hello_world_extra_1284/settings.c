#include "settings.h"

int32_t settings[NUM_SETTINGS];
float settings_float[NUM_SET_FLOAT];
int32_t settings_int[NUM_SET_INT];
uint8_t settings_type[NUM_SETTINGS]; //0 for float, 1 for int
uint8_t settings_idx[NUM_SETTINGS]; //index in the specific array
char msg_settings[100];

float get_settings_value_float(int32_t code)
{
    return settings_float[ settings_idx[code]];
}

int32_t get_settings_value_int(int32_t code)
{
    return settings_int[ settings_idx[code]];
}

void update_settings_type_values(){
    uint8_t i;
    char msg[20];
    for(i=0; i< NUM_SETTINGS; i++) {
        if(settings_type[i]) {
            settings_int[ settings_idx[i]] = settings[i]/10000;
        }
        else
            UPDATE_SETTING(settings_float[ settings_idx[i]], settings[i]);
    }
}

void initialize_default()
{
    settings[SENTINEL_POS] = 0;
    settings[EAX_POS] = 5020;
    settings[EAY_POS] = -2500;
    settings[EAZ_POS] = 5500;
    settings[EGX_POS] = 22130;
    settings[EGY_POS] = -2790;
    settings[EGZ_POS] = 770;
    settings[EMX_POS] = 810000;
    settings[EMY_POS] = 2670000;
    settings[EMZ_POS] = -1240000;
    settings[KP_POS] = 5000;
    settings[KI_POS] = 5000;
    settings[KD_POS] = 5000;
    settings[CARMA_POS] = 15000000;
    settings[WING_FLAPS_POS] = 15000000;
    settings[MOTOR_RETURN_POWER_POS] = 15000000;
    settings[RETURN_HOME_DISTANCE_POS] = 100000;
}

void initialize_settings()
{
    int32_t sentinel_val;
    uint8_t i;
    sentinel_val = eeprom_read_dword((uint32_t*)(SENTINEL_POS<<2));
    if(sentinel_val == EEPROM_SENTINEL) 
    {
        for(i=0;i<NUM_SETTINGS;i++)
            settings[i] = eeprom_read_dword((uint32_t*)(i<<2));
    }
    else initialize_default();

    settings_type[SENTINEL_POS] = 1;
    settings_idx[SENTINEL_POS] = 0;

    settings_type[EAX_POS] = 0;
    settings_idx[EAX_POS] = 0;

    settings_type[EAY_POS] = 0;
    settings_idx[EAY_POS] = 1;

    settings_type[EAZ_POS] = 0;
    settings_idx[EAZ_POS] = 2;

    settings_type[EGX_POS] = 0;
    settings_idx[EGX_POS] = 3;

    settings_type[EGY_POS] = 0;
    settings_idx[EGY_POS] = 4;

    settings_type[EGZ_POS] = 0;
    settings_idx[EGZ_POS] = 5;

    settings_type[EMX_POS] = 1;
    settings_idx[EMX_POS] = 1;

    settings_type[EMY_POS] = 1;
    settings_idx[EMY_POS] = 2;

    settings_type[EMZ_POS] = 1;
    settings_idx[EMZ_POS] = 3;

    settings_type[KP_POS] = 0;
    settings_idx[KP_POS] = 6;

    settings_type[KI_POS] = 0;
    settings_idx[KI_POS] = 7;
    
    settings_type[KD_POS] = 0;
    settings_idx[KD_POS] = 8;

    settings_type[CARMA_POS] = 1;
    settings_idx[CARMA_POS] = 4;

    settings_type[WING_FLAPS_POS] = 1;
    settings_idx[WING_FLAPS_POS] = 5;

    settings_type[MOTOR_RETURN_POWER_POS] = 1;
    settings_idx[MOTOR_RETURN_POWER_POS] = 6;

    settings_type[RETURN_HOME_DISTANCE_POS] = 0;
    settings_idx[RETURN_HOME_DISTANCE_POS] = 9;

    update_settings_type_values();
}

void show_list()
{
    uint8_t i;
    msg_settings[0]=0;
    append_command(msg_settings,5);
    for(i=0; i<NUM_SETTINGS; i++) 
    {
        append_int(msg_settings,i);
        append_int(msg_settings, settings[i]);
        if(i%4 == 3)
        {
            USART0_print(msg_settings);
            msg_settings[0]=0;
        }
    }
    strcat(msg_settings,"\n");
    USART0_print(msg_settings);
}

void update_setting(int32_t code, int32_t value) 
{
    settings[code] = value;
    if(settings_type[code])
        settings_int[ settings_idx[code]] = value/10000;
    else
        UPDATE_SETTING(settings_float[ settings_idx[code]], value)
}


void save_setting()
{
    int32_t i;
    for(i=0; i<NUM_SETTINGS; i++) 
        eeprom_write_dword( (uint32_t*)(i<<2),settings[i]);
}