# PORT ?= /dev/ttyUSB0
PORT ?= COM3
# CFLAGS=-Wl,-u,vfprintf -lprintf_flt -lm
CFLAGS=-lc -lm

GCCFLAGS=-g -Os -Wall -Wextra -mmcu=atmega1284p 
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt #-lm
LINKOBJECTS= -lc -lm


all: hello_world.hex

program: upload

hello_world.elf: *.c *.h
	avr-g++ $(CFLAGS) -mmcu=atmega1284p -DF_CPU=16000000 -O3 -Wall -Wextra -o $@ $^ -lc

hello_world.hex: hello_world.elf
	avr-objcopy -j .text -j .data -O ihex $^ $@
	avr-size $@

upload: hello_world.hex
	avrdude -c arduino -P $(PORT) -b 115200 -p atmega1284 -U flash:w:hello_world.hex:i
	
clean:
	rm -rf hello_world.elf hello_world.hex

.PHONY: all clean program upload
