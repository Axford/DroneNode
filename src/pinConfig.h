/*
*
*  Pin mappings for the robot boat motherboard
*
*/

#include <Arduino.h>

#define PIN_LED   2

/*
*   Motherboard mappings
*/

// Serial headers
#define PIN_SERIAL0_TX   1
#define PIN_SERIAL0_RX   3

#define PIN_SERIAL1_TX   12
#define PIN_SERIAL1_RX   13

#define PIN_SERIAL2_TX   17
#define PIN_SERIAL2_RX   16


// General IO headers
#define PIN_OUT0_0  32 // ADC1
#define PIN_OUT0_1  33 // ADC1

#define PIN_OUT1_0  14 // ADC2 - can't be used with wifi
#define PIN_OUT1_1  15 // ADC2 - can't be used with wifi

#define PIN_OUT2_0  2  // ADC2 - can't be used with wifi
#define PIN_OUT2_1  4  // ADC2 - can't be used with wifi

#define PIN_DAC0_0  25 // ADC2 - can't be used with wifi
#define PIN_DAC0_1  26 // ADC2 - can't be used with wifi

#define PIN_IN0_0  34  // input only, ADC1
#define PIN_IN0_1  35  // input only, ADC1

// SD Card header
//#define PIN_SD_1   GND
//#define PIN_SD_2   VCC
#define PIN_SD_3   23  // MOSI
#define PIN_SD_4   19  // MISO
#define PIN_SD_5   18  // CLK
#define PIN_SD_6   5   // CSO


// I2C multiplexer
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define PIN_I2C_RESET 27
