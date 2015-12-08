# JTQuad

A quad-rotor helicopter drone built around an AVR XMEGA microcontroller and Micromega FPU.

* 9 degrees for freedom provided by a 3-axis accelerometer, gyroscope and magnetometer
* Radio controlled (PWM receiver)
* 2 way USB to serial communications
* Calibration routines and a safety lock

## Hardware

|Component             |Product                                              |
|:---------------------|-----------------------------------------------------|
|MCU                   |ATXMega128A1 AU with 16MHZ external crystal (runs at 32MHz with PLL)|
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
|

## Pin Assignment

|Pin   | Peripheral Description   |
|------|--------------------------|
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


## Timers


