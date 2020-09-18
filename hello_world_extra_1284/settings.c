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
    for(i=0; i< NUM_SETTINGS; i++) {
        if(settings_type[i]) {
            settings_int[ settings_idx[i]] = settings[i];
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
    settings[EMX_POS] = 81;
    settings[EMY_POS] = 267;
    settings[EMZ_POS] = -124;
    settings[KP_POS] = 5000;
    settings[KI_POS] = 1000;
    settings[KD_POS] = 0;
    settings[CARMA_POS] = 1500;
    settings[WING_FLAPS_POS] = 1500;
    settings[MOTOR_RETURN_POWER_POS] = 1500;
    settings[RETURN_HOME_DISTANCE_POS] = 500000;
    settings[ALPHA_CARMA_PID] = 9000;
    settings[ALPHA_CARMA_SERVO] = 9000;
    settings[ALPHA_WINGS_SERVO] = 9000;
    settings[ALPHA_ESC] = 9000;
    settings[ALPHA_PUMP_SOFT_START] = 9000;
    settings[PUMP_DUTY_CYCLE] = 25; 
    settings[STEPPER_MAX_VALUE] = 38000;
    settings[MOTOR_MAX_TEMP] = 70;
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

    settings_type[ALPHA_CARMA_PID] = 0;
    settings_idx[ALPHA_CARMA_PID] = 10;


    settings_type[ALPHA_CARMA_SERVO] = 0;
    settings_idx[ALPHA_CARMA_SERVO] = 11;

    settings_type[ALPHA_WINGS_SERVO] = 0;
    settings_idx[ALPHA_WINGS_SERVO] = 12;

    settings_type[ALPHA_ESC] = 0;
    settings_idx[ALPHA_ESC] = 13;

    settings_type[ALPHA_PUMP_SOFT_START] = 0;
    settings_idx[ALPHA_PUMP_SOFT_START] = 14;

    settings_type[PUMP_DUTY_CYCLE] = 1;
    settings_idx[PUMP_DUTY_CYCLE] = 7;

    settings_type[STEPPER_MAX_VALUE] = 1;
    settings_idx[STEPPER_MAX_VALUE] = 8;

    settings_type[MOTOR_MAX_TEMP] = 0;
    settings_idx[MOTOR_MAX_TEMP] = 15;

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
    strcat(msg_settings,",");
    USART0_print(msg_settings);

    msg_settings[0]=0;
    for (i = 0; i < NUM_SETTINGS; i++)
    {
        char *res = (char *)pgm_read_word(&(settings_labels[i]));
        strcpy_P(msg_settings, res);
        if( i < NUM_SETTINGS - 1)
            strcat(msg_settings, ",");
        USART0_print(msg_settings);
    }
    USART0_print("\n");
}

void update_setting(int32_t code, int32_t value) 
{
    settings[code] = value;
    if(settings_type[code])
        settings_int[ settings_idx[code]] = value;
    else
        UPDATE_SETTING(settings_float[ settings_idx[code]], value)
}

void save_setting()
{
    int32_t i;
    for(i=0; i<NUM_SETTINGS; i++) 
        eeprom_write_dword( (uint32_t*)(i<<2),settings[i]);
}

const char settings_label_1[] = "SENTINEL";
const char settings_label_2[] = "EAX";
const char settings_label_3[] = "EAY";
const char settings_label_4[] = "EAZ";

const char settings_label_5[] = "EGX";
const char settings_label_6[] = "EGY";
const char settings_label_7[] = "EGZ";

const char settings_label_8[] = "EMX";
const char settings_label_9[] = "EMY";
const char settings_label_10[] = "EMZ";

const char settings_label_11[] = "KP";
const char settings_label_12[] = "KI";
const char settings_label_13[] = "KD";

const char settings_label_14[] = "CARMA_BIAS";
const char settings_label_15[] = "WING_FLAPS_BIAS";
const char settings_label_16[] = "MOTOR_RETURN_HOME_POWER";

const char settings_label_17[] = "RETURN_HOME_DISTANCE";

const char settings_label_18[] = "ALPHA_CARMA_PID";
const char settings_label_19[] = "ALPHA_CARMA_SERVO";
const char settings_label_20[] = "ALPHA_WINGS_SERVO";
const char settings_label_21[] = "ALPHA_ESC";
const char settings_label_22[] = "ALPHA_PUMP_SOFT_START";

const char settings_label_23[] = "PUMP_DUTY_CYCLE";
const char settings_label_24[] = "STEPPER_MAX_CYCLES";
const char settings_label_25[] = "MOTOR_MAX_TEMPERATURE";

const char *const settings_labels[] =
    {

        settings_label_1,
        settings_label_2,
        settings_label_3,
        settings_label_4,

        settings_label_5,
        settings_label_6,

        settings_label_7,
        settings_label_8,
        settings_label_9,

        settings_label_10,
        settings_label_11,
        settings_label_12,

        settings_label_13,
        settings_label_14,

        settings_label_15,
        settings_label_16,
        settings_label_17,
        settings_label_18,

        settings_label_19,

        settings_label_20,

        settings_label_21,

        settings_label_22,

        settings_label_23,
        settings_label_24,
        settings_label_25};
