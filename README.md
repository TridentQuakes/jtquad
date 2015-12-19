# JTQuad

Flight software, written in C and assembly, is processed on an AVR XMEGA microcontroller and aided by a Micromega FPU.

* 9 degrees of freedom provided by three-dimensional accelerometer, gyroscope and magnetometer
* Radio controlled (PWM receiver)
* 2 way USB to serial communications
* Calibration routines and a safety lock

## Hardware

|Component             |Product                                              |
|:---------------------|:----------------------------------------------------|
|MCU                   |ATXMega128A1 AU with 16MHz external crystal (runs at 32MHz with PLL)|
|FPU                   |UM-FPU v3.1|
|Accelerometer         |ADXL345|                             
|Gyroscope             |ITG3200|
|Magnetometer          |HMC5883L or HMC5843|
|Thermometer           |TMP102|
|USB to Serial         |FTDI IC|
|Radio Controller      |Specktrum DX7|
|Radio Receiver        |AR7000 receiver outputs PWM signal|
|3.3V Power Supply     |Custom design, powers MCU, FPU and sensors|
|5V Power Supply       |Custom design, powers radio receiver and the TTL converters|
|Motors                |4 x Turnigy 2217 (16 turn 1050Kv)|
|ESCs                  |4 x GWESC35A (Brushless 35 amp ESC with BEC)|
|Propellers            |2 x APC 10x4.7 Slow Flyer Pusher Propellers (clockwise, APC10047SFP)|
|Propellers            |APC 10x4.7 Slow Flyer Propellers (counter-clockwise, APC10047SF)|

## Pin Assignment

|Pin   |Peripheral Description   |
|:-----|:------------------------|
|PA0|AR7000, channel 0, CCA
|PA1|AR7000, channel 1, CCB
|PA2|AR7000, channel 2,	CCC
|PA3|AR7000, channel 3, CCD
|PA4|
|PA5|
|PA6|
|PA7|
|PB0|AR7000, channel 4, CCA
|PB1|AR7000, channel 5, CCB
|PB2|AR7000, channel 6, CCC
|PB3|
|PB4|
|PB5|
|PB6|
|PB7|
|PC0|TMP102, TWI
|PC1|TMP102, TWI
|PC2|FTDI, serial communications
|PC3|FTDI, serial communications
|PC4|
|PC5|		
|PC6|		
|PC7|
|PD0|HMC5883L, TWI
|PD1|HMC5883L, TWI
|PD2|Extra serial communications port (not in use)
|PD3|Extra serial communications port (not in use)
|PD4|ADXL345, SPI (4-wire)
|PD5|ADXL345, SPI (4-wire)
|PD6|ADXL345, SPI (4-wire)
|PD7|ADXL345, SPI (4-wire)
|PE0|ITG3200, TWI
|PE1|ITG3200, TWI
|PE2|		
|PE3|		
|PE4|UM-FPU v3.1, SPI (4-wire)
|PE5|UM-FPU v3.1, SPI (4-wire)
|PE6|UM-FPU v3.1, SPI (4-wire)
|PE7|UM-FPU v3.1, SPI (4-wire)
|PF0|Motor 0
|PF1|Motor 1
|PF2|Motor 2
|PF3|Motor 3
|PF4|		
|PF5|		
|PF6|		
|PF7|		
|PH0|
|PH1|
|PH2|
|PH3|
|PH4|
|PH5|
|PH6|
|PH7|

## Serial Commands

### Write

|Command	Action|Decription|
|:--------------|:---------|
X|Stop sending messages

### Read

|Command|Description              |Output Format |
|:------|:------------------------|:-------------|
=|Reserved debug command|
e|Send min_armed_throttle|
f|Send_transmitter_smooth_factor|receiver_smooth_factor[1,2,3,4,5,6,7],dummy,dummy,dummy
g|Send transmitter_slope|receiver_slope[1,2,3,4,5,6,7]
h|Send_transmitter_offset|receiver_offset[1,2,3,4,5,6,7]
i|Send sensor data|gyro(x,y,z),accel(x,y,z),mag(x,y,z)
j|Send mag raw sensor data|x,y,z	
k|Send accel calibration data|offset(x,y,z),scale(x,y,z)
l|Send accel raw sensor data|x,y,z
m|Send mag calibration data|offset(x,y,z),scale(x,y,z)
n|Send gyro calibration data|offset(x,y,z),scale
o|
p|
q|
r|Send vehicle attitude|kinematics_angle(x,y),heading
s|Send all flight data|motors_armed,attitude(x,y,z),dummy,dummy,receiver_command(1,2,3,4,5,6,7),dummy,motor_command(1,2,3,4),dummy,dummy,dummy,dummy,dummy,flight_mode
t|Send transmitter data|receiver_command[1,2,3,4,5,6,7]
x|Stop sending messages|
!|Send flight software version|

## Timers

|Timer  |Description                        |
|:------|:----------------------------------|
TCC0|Used as a one second LED heartbeat
TCC1|Used to implement the micros() function
TCD0|Used for input capture of channels 0, 1, 2 and 3 on pins PA0, PA1, PA2 and PA3 respectively
TCD1|Used to maintain a 400Hz application loop
TCE0|Used for input capture of channels 4, 5 and 6 on pins PB0, PB1 and PB2 respectively
TCE1|
TCF0|Used for PWM output to the motor ESCs
TCF1|
