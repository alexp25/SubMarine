#include "serial_utils.h"

void serial0_print_int(int val)
{
    char buf[10];
    sprintf(buf, "%d", val);
    USART0_print(buf);
}

void serial0_print_float(double val)
{
    char buf[10];
    sprintf(buf, "%f", val);
    USART0_print(buf);
}

void serial0_print_kv(char *key, int32_t val, uint8_t eol)
{
    char buf[10];
    if (eol)
    {
        sprintf(buf, ": %ld\n", val);
    }
    else
    {
        sprintf(buf, ": %ld", val);
    }
    USART0_print(key);
    USART0_print(buf);
}

void serial0_print_kvx(char *key, int32_t val, char *format, uint8_t eol)
{
    char buf[10];
    char bformat[10];
    strcpy(bformat, ": ");
    strcat(bformat, format);

    if (eol)
    {
        strcat(bformat, "\n");
    }

    sprintf(buf, bformat, val);

    USART0_print(key);
    USART0_print(buf);
}

void serial0_print_kvx_float(char *key, double val, char *format, uint8_t eol)
{
    char buf[10];
    char bformat[10];
    strcpy(bformat, ": ");
    strcat(bformat, format);

    if (eol)
    {
        strcat(bformat, "\n");
    }

    sprintf(buf, bformat, val);

    USART0_print(key);
    USART0_print(buf);
}