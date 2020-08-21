#include "string_utils.h"
char buff[10];

void append_float(char *dest, float data)
{
    int nr = (int)data;

    if (data < 0)
        sprintf(buff, ",%d.%d", nr, nr * 100 - (int)(data * 100));
    else
        sprintf(buff, ",%d.%d", nr, (int)(data * 100) - nr * 100);

    strcat(dest, buff);
}

void append_command(char *dest, int data)
{
    sprintf(buff, "%d", data);
    strcat(dest, buff);
}

void append_int(char *dest, uint32_t data)
{
    sprintf(buff, ",%ld", data);
    strcat(dest, buff);
}
