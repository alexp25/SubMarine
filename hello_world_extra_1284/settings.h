#include <avr/eeprom.h>
#include "serial.h"
#include "string_utils.h"

#define UPDATE_SETTING(x,y) x = ((float)y)/10000;

#define EEPROM_SENTINEL 0xFA
#define EEPROM_DEFAULTS 0x00
#define NUM_SETTINGS 17
#define NUM_SET_FLOAT 10
#define NUM_SET_INT 7
extern int32_t settings[NUM_SETTINGS];
extern float settings_float[NUM_SET_FLOAT];
extern int32_t settings_int[NUM_SET_INT];


void initialize_settings();
void initialize_default();
void show_list();
void update_setting(int32_t code, int32_t value);
void save_setting();

float get_settings_value_float(int32_t code);
int32_t get_settings_value_int(int32_t code);

//list of codes for eeprom memory
#define SENTINEL_POS 0
#define EAX_POS 1
#define EAY_POS 2
#define EAZ_POS 3
#define EGX_POS 4
#define EGY_POS 5
#define EGZ_POS 6
#define EMX_POS 7
#define EMY_POS 8
#define EMZ_POS 9
#define KP_POS 10
#define KI_POS 11
#define KD_POS 12
#define CARMA_POS 13
#define WING_FLAPS_POS 14
#define MOTOR_RETURN_POWER_POS 15
#define RETURN_HOME_DISTANCE_POS 16