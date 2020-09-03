#include <avr/eeprom.h>
#include "serial.h"
#include "string_utils.h"
#include <avr/pgmspace.h>

#define UPDATE_SETTING(x,y) x = ((float)y)/10000;

#define EEPROM_SENTINEL 0xFA
#define EEPROM_DEFAULTS 0x00
#define NUM_SETTINGS 23
#define NUM_SET_FLOAT 15
#define NUM_SET_INT 8
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
uint8_t print_labels(uint8_t page, char *str);

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
#define ALPHA_CARMA_PID 17
#define ALPHA_CARMA_SERVO 18
#define ALPHA_WINGS_SERVO 19
#define ALPHA_ESC 20
#define ALPHA_PUMP_SOFT_START 21
#define PUMP_DUTY_CYCLE 22

extern const char settings_label_1[] PROGMEM;
extern const char settings_label_2[] PROGMEM;
extern const char settings_label_3[] PROGMEM;
extern const char settings_label_4[] PROGMEM;

extern const char settings_label_5[] PROGMEM;
extern const char settings_label_6[] PROGMEM;

extern const char settings_label_7[] PROGMEM;
extern const char settings_label_8[] PROGMEM;
extern const char settings_label_9[] PROGMEM;

extern const char settings_label_10[] PROGMEM;
extern const char settings_label_11[] PROGMEM;
extern const char settings_label_12[] PROGMEM;

extern const char settings_label_13[] PROGMEM;
extern const char settings_label_14[] PROGMEM;

extern const char settings_label_15[] PROGMEM;
extern const char settings_label_16[] PROGMEM;
extern const char settings_label_17[] PROGMEM;
extern const char settings_label_18[] PROGMEM;

extern const char settings_label_19[] PROGMEM;

extern const char settings_label_20[] PROGMEM;

extern const char settings_label_21[] PROGMEM;

extern const char settings_label_22[] PROGMEM;
extern const char settings_label_23[] PROGMEM;


extern const char *const settings_labels[] PROGMEM;
