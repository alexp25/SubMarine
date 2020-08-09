#ifndef __SERIAL_UTILS_H__
#define __SERIAL_UTILS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include "serial.h"

    void serial0_print_int(int val);

    void serial0_print_float(double val);

    void serial0_print_kv(char *key, int32_t val, uint8_t eol);

    void serial0_print_kvx(char *key, int32_t val, char *format, uint8_t eol);

    void serial0_print_kvx_float(char *key, double val, char *format, uint8_t eol);

#ifdef __cplusplus
}
#endif

#endif