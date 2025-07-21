/***************************************************
 This is a library written for the Maxim MAX30102

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.

 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.

 Library modified to interface with RPi Pico's I2C library
 Removed variables, functions, and definitions specific to MAX30105 functionality,
 particularly relating to the code configuring the unused green LED hardware
 --George Nassour 4/28/2025
 *****************************************************/

// Std Library Header
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Pico Defined Headers
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

// Max30102 Driver Headers
#include "MAX30102.h"

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

// UART defines
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1


uint32_t irBuffer[100];     // infrared LED sensor data
uint32_t redBuffer[100];    // red LED sensor data
int32_t bufferLength = 100; // data length
int32_t spo2;               // SPO2 value
int8_t validSPO2;           // indicator to show if the SPO2 calculation is valid
int32_t heartRate;          // heart rate value
int8_t validHeartRate;      // indicator to show if the heart rate calculation is valid
byte pulseLED = 11;         // Must be on PWM pin
byte readLED = 13;          // Blinks with each data read

int main()
{
  int ack;

  size_t nBytes;

  // Debug functions init (USB serial & onboard LED)
  stdio_init_all();
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  // // Set up our UART
  uart_init(UART_ID, BAUD_RATE);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  // I2C Init
  i2c_init(I2C_PORT, 100 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);

  // Pulse ox init
  init(I2C_PORT, I2C_SPEED_FAST, MAX30102_ADDRESS);

  // Pulse Ox setup initial values
  byte ledBrightness = 80; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;    // Options: 69, 118, 215, 411
  int adcRange = 16384;    // Options: 2048, 4096, 8192, 16384

  setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  while (true)
  {

    while (available() == 0) // do we have new data?
      check();               // Check the sensor for new data

    nextSample(); // We're finished with this sample so move to next sample
  }
}
