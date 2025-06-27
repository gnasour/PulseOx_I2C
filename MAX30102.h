



/*************************************************** 
 This is a library written for the Maxim MAX30105 Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.
 
 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.

 Library modified to interface with RPi Pico's I2C library
 --George Nassour 4/28/2025
 *****************************************************/
#pragma once

#ifndef MAX30102_H
#define MAX30102_H

#include "hardware/i2c.h"

#define MAX30102_ADDRESS          0x57

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000
#define I2C_BUFFER_LENGTH         6       // 2*3bytes per LED channel (IR+RED)
                                          // Depending on the ADC config, 15-18 bits are recorded, which requires 3 bytes per reading

#define delay(delay_ms) busy_wait_ms(delay_ms)
#define millis() time_us_32()

//Useful typedefs between arduino and rpi
typedef uint8_t byte;
typedef i2c_inst_t TwoWire;

// Status Registers
static const uint8_t MAX30102_INTSTAT1 =		0x00;
static const uint8_t MAX30102_INTSTAT2 =		0x01;
static const uint8_t MAX30102_INTENABLE1 =		0x02;
static const uint8_t MAX30102_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30102_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30102_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30102_FIFOREADPTR = 	0x06;
static const uint8_t MAX30102_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30102_FIFOCONFIG = 		0x08;
static const uint8_t MAX30102_MODECONFIG = 		0x09;
static const uint8_t MAX30102_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30102_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30102_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30102_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30102_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30102_DIETEMPINT = 		0x1F;
static const uint8_t MAX30102_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30102_DIETEMPCONFIG = 	0x21;

// Part ID Registers
static const uint8_t MAX30102_REVISIONID = 		0xFE;
static const uint8_t MAX30102_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30102_INT_A_FULL_MASK =		(byte)0b10000000;
static const uint8_t MAX30102_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30102_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30102_INT_DATA_RDY_MASK = (byte)0b01000000;
static const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_INT_ALC_OVF_MASK = (byte)0b00100000;
static const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30102_INT_PROX_INT_MASK = (byte)0b00010000;
static const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = (byte)0b00000010;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_SAMPLEAVG_MASK =	(byte)0b11100000;
static const uint8_t MAX30102_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30102_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30102_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30102_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30102_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30102_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30102_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30102_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30102_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30102_SHUTDOWN = 		0x80;
static const uint8_t MAX30102_WAKEUP = 			0x00;

static const uint8_t MAX30102_RESET_MASK = 		0xBF;
static const uint8_t MAX30102_RESET = 			0x40;

static const uint8_t MAX30102_MODE_MASK = 		0xF8;
static const uint8_t MAX30102_MODE_REDONLY = 	0x02;
static const uint8_t MAX30102_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30102_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30102_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30102_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30102_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30102_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30102_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30102_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30102_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30102_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30102_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30102_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30102_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30102_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30102_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30102_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30102_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30102_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30102_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30102_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30102_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
//Slots should be enabled in order, so SLOT1 should not be enabled
//if SLOT2 is
static const uint8_t MAX30102_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30102_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;

static const uint8_t MAX_30102_EXPECTEDPARTID = 0x15U;

bool init(TwoWire *wire, uint32_t i2cSpeed, uint8_t i2caddr);

uint32_t getRed(void); //Returns immediate red value
uint32_t getIR(void); //Returns immediate IR value
uint32_t getGreen(void); //Returns immediate green value
bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

// Configuration
void softReset();
void shutDown(); 
void wakeUp(); 

void setLEDMode(uint8_t mode);

void setADCRange(uint8_t adcRange);
void setSampleRate(uint8_t sampleRate);
void setPulseWidth(uint8_t pulseWidth);

void setPulseAmplitudeRed(uint8_t value);
void setPulseAmplitudeIR(uint8_t value);
void setPulseAmplitudeGreen(uint8_t value);
void setPulseAmplitudeProximity(uint8_t value);

void setProximityThreshold(uint8_t threshMSB);

//Multi-led configuration mode (page 22)
void enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
void disableSlots(void);

// Data Collection

//Interrupts (page 13, 14)
uint8_t getINT1(void); //Returns the main interrupt group
uint8_t getINT2(void); //Returns the temp ready interrupt
void enableAFULL(void); //Enable/disable individual interrupts
void disableAFULL(void);
void enableDATARDY(void);
void disableDATARDY(void);
void enableALCOVF(void);
void disableALCOVF(void);
void enablePROXINT(void);
void disablePROXINT(void);
void enableDIETEMPRDY(void);
void disableDIETEMPRDY(void);

//FIFO Configuration (page 18)
void setFIFOAverage(uint8_t samples);
void enableFIFORollover();
void disableFIFORollover();
void setFIFOAlmostFull(uint8_t samples);

//FIFO Reading
uint16_t check(void); //Checks for new data and fills FIFO
uint8_t available(void); //Tells caller how many new samples are available (head - tail)
void nextSample(void); //Advances the tail of the sense array
uint32_t getFIFORed(void); //Returns the FIFO sample pointed to by tail
uint32_t getFIFOIR(void); //Returns the FIFO sample pointed to by tail
uint32_t getFIFOGreen(void); //Returns the FIFO sample pointed to by tail

uint8_t getWritePointer(void);
uint8_t getReadPointer(void);
void clearFIFO(void); //Sets the read/write pointers to zero

//Proximity Mode Interrupt Threshold
void setPROXINTTHRESH(uint8_t val);

// Die Temperature
float readTemperature();
float readTemperatureF();

// Detecting ID/Revision
uint8_t getRevisionID();
uint8_t readPartID();  

// Setup the IC with user selectable settings
void setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange);

// Low-level I2C communication
uint8_t readRegister8(uint8_t address, uint8_t reg);
void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
uint8_t _i2caddr;

//activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
byte activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

uint8_t revisionID; 

void readRevisionID();

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
typedef struct Record
{
uint32_t red[STORAGE_SIZE];
uint32_t IR[STORAGE_SIZE];
uint32_t green[STORAGE_SIZE];
byte head;
byte tail;
} sense_struct; //This is our circular buffer of readings from the sensor

sense_struct sense;

#endif