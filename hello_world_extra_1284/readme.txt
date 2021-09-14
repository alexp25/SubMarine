# add atmega1284 definition for avrdude
see readme in root folder

# usb programming & serial
connect usb adapter to USB_TTL header

# upload hex
press reset (RST jumper)
make upload
release reset

# power supply description
VCC: power MCU, bluetooth HC-05, GPS
VCCS: power other sensors 3v3/5v (MPU6050, BMP280, ESP8266), recommended 3v3
VCC1: power from ESC1, designed to power sensors and MCUs 
VCC2: power from ESC2, designed to power servos (S1, S2, S3, S4)

# power setup default (recommended)
PS_SEN: connect VCCS to 3v3 (jumper) to power 3v3 sensors (MPU6050, BMP280, ESP8266)
PS_VCC: connect VCC to VCC1 (jumper)

1. add 1284 definition (avrdude.add.conf) at the end of avrdude.conf
located in C:\WinAVR-20100110\bin\avrdude.conf

2. reset before uploading with avrdude

avr setup:
fuses (extended high low) 0xFF 0x98 0xFF
lock bits 0xFF
