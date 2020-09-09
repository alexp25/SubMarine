#ifndef __SERIAL_PARSER_H__
#define __SERIAL_PARSER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "serial.h"

#define N_SERIAL_DATA_STR 100
#define N_SERIAL_DATA_VAL 100

    extern char inData[N_SERIAL_DATA_STR];
    extern long dataArray[N_SERIAL_DATA_VAL];
    extern uint8_t incomplete_line;
    extern uint8_t index_serial_data;

    int parse_csv(char *inputString, long *outputArray, int outputArraySize);
    void check_com_raw(void (*onreceive)(char *));
    void check_com(void (*onparse)(int, long *, int));

#ifdef __cplusplus
}
#endif

#endif
