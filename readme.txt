1. add 1284 definition (avrdude.add.conf) at the end of avrdude.conf
located in C:\WinAVR-20100110\bin\avrdude.conf

2. reset before uploading with avrdude

avr setup:
fuses (extended high low) 0xFF 0x98 0xFF
lock bits 0xFF