#include "serial_parser_c.h"


char inData[N_SERIAL_DATA_STR];
long dataArray[N_SERIAL_DATA_VAL];
uint8_t incomplete_line = 0;
uint8_t index_serial_data = 0;

int parse_csv(char *inputString, long *outputArray, int outputArraySize)
{
    char *pch;
    long val = 0;
    int index_serial_data = 0;
    pch = strtok(inputString, ",");

    while (pch != NULL && pch != "\n")
    {
        sscanf(pch, "%ld", &val);
        // val = String(pch).toInt();
        outputArray[index_serial_data] = val;
        index_serial_data++;
        if (index_serial_data == outputArraySize)
        {
            break;
        }
        pch = strtok(NULL, ",");
    }
    return index_serial_data;
}

void check_com_raw(void (*onreceive)(char *))
{
    if (USART0_available() > 0)
    {
        char aChar = USART0_receive_buffered();
        inData[index_serial_data] = aChar;
        index_serial_data++;
        if (aChar == '\n' || index_serial_data == N_SERIAL_DATA_STR - 1)
        {
            /**
             if previously there was an incomplete line
             ignore the line AND the next line (until the next \n)
          */
            uint8_t read_line = 1;
            if (incomplete_line)
            {
                incomplete_line = 0;
                read_line = 0;
            }
            if (aChar != '\n')
            {
                incomplete_line = 1;
                read_line = 0;
            }
            inData[index_serial_data] = '\0';

            if (read_line)
            {
                (*onreceive)(inData);
            }
            else
            {
            }

            inData[index_serial_data] = '\0'; // Keep the string NULL terminated
            index_serial_data = 0;
            inData[0] = '\0';
        }
    }
}

void check_com(void (*onparse)(int, long *, int))
{
    if (USART0_available() > 0)
    {
        char aChar = USART0_receive_buffered();
        inData[index_serial_data] = aChar;
        index_serial_data++;

        if (aChar == '\n' || index_serial_data == N_SERIAL_DATA_STR - 1)
        {
            /**
                if previously there was an incomplete line
                ignore the line AND the next line (until the next \n)
            */
            uint8_t read_line = 1;
            if (incomplete_line)
            {
                incomplete_line = 0;
                read_line = 0;
            }
            if (aChar != '\n')
            {
                incomplete_line = 1;
                read_line = 0;
            }
            inData[index_serial_data] = '\0';

            if (read_line)
            {
                int nparse = parse_csv(inData, dataArray, N_SERIAL_DATA_VAL);
                int cmd = dataArray[0];
                long checksum = 0;

                for (int i = 0; i < nparse - 1; i++)
                {
                    checksum += dataArray[i];
                }

                if (checksum != dataArray[nparse - 1])
                {
                    nparse = -1;
                }

                (*onparse)(cmd, dataArray, nparse);
            }
            else
            {
            }

            inData[index_serial_data] = '\0'; // Keep the string NULL terminated
            index_serial_data = 0;
            inData[0] = '\0';
        }
    }
}
