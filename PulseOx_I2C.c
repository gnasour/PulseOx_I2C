/*************************************************** 
 This is a library written for the Maxim MAX30102 Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.
 
 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.

 Library modified to interface with RPi Pico's I2C library
 Removed variables, functions, and definitions specific to MAX30102 functionality
 --George Nassour 4/28/2025
 *****************************************************/

//Pico Defined Headers
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

//Driver Headers
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


uint8_t readByte;  
uint8_t readByteStream[I2C_BUFFER_LENGTH];
uint8_t writePacket[2]; // writePacket[0] = device reg addr, writePacket[1] = val 


void panic_blink(){
    while(true){
      gpio_put(PICO_DEFAULT_LED_PIN, true);
      sleep_ms(200);
      gpio_put(PICO_DEFAULT_LED_PIN, false);
      sleep_ms(200);
    }
}

//
//Initialization
//

bool max30102_init(TwoWire *wirePort, uint32_t i2cSpeed, uint8_t i2caddr) {

    _i2cPort = wirePort; //Grab which port the user wants us to use

    _i2caddr = i2caddr;

    // Step 1: Initial Communication and Verification
    // Check that a MAX30102 is connected
    if (readPartID() != 0x15) {
        // Error -- Part ID read from MAX30102 does not match expected part ID.
        // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        return false;
    }

    return true;
}

//
//End Initialization
//


//
//Power States
//

void softReset(void) {
  
    bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);
  
    // Poll for bit to clear, reset is then complete
    // Timeout after 100ms
    unsigned long startTime = millis();
    while (millis() - startTime < 100)
    {
      uint8_t response = readRegister8(_i2caddr, MAX30102_MODECONFIG);
      if ((response & MAX30102_RESET) == 0) break; //We're done!
      delay(1); //Let's not over burden the I2C bus
    }
}

void shutDown(void)
{
    // Put IC into low power mode (datasheet pg. 19)
    // During shutdown the IC will continue to respond to I2C commands but will
    // not update with or take new readings (such as temperature)
    bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void wakeUp(void) 
{
    // Pull IC out of low power mode (datasheet pg. 19)
    bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

//
// End Power States
//

//
// Interrupt Configuration
//

uint8_t getINT1(void) {
    return (readRegister8(_i2caddr, MAX30102_INTSTAT1));
}
uint8_t getINT2(void) {
  return (readRegister8(_i2caddr, MAX30102_INTSTAT2));
}

void enableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}
void disableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void enableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void disableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void enableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void disableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void enablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}
void disablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void enableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}
void disableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

//
// End Interrupt configuration
//

//
// Mode Configuration
//

void setLEDMode(uint8_t mode) 
{
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void setADCRange(uint8_t adcRange) 
{
  // adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

void setSampleRate(uint8_t sampleRate) 
{
  // sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30102_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED2_PULSEAMP, amplitude);
}


//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void enableSlot(uint8_t slotNumber, uint8_t device) {

  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments
void disableSlots(void) {
  writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG1, 0);
  writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG2, 0);
}

//
// End Mode Configuration
//


//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void clearFIFO(void) {
  writeRegister8(_i2caddr, MAX30102_FIFOWRITEPTR, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOOVERFLOW, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void enableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void disableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t getWritePointer(void) {
  return (readRegister8(_i2caddr, MAX30102_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t getReadPointer(void) {
  return (readRegister8(_i2caddr, MAX30102_FIFOREADPTR));
}

//
// End FIFO Configuration
//

// Die Temperature
// Returns temp in C
float readTemperature() {
	
  //DIE_TEMP_RDY interrupt must be enabled
  //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19
  
  // Step 1: Config die temperature register to take 1 temperature sample
  writeRegister8(_i2caddr, MAX30102_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    //uint8_t response = readRegister8(_i2caddr, MAX30102_DIETEMPCONFIG); //Original way
    //if ((response & 0x01) == 0) break; //We're done!
    
	//Check to see if DIE_TEMP_RDY interrupt is set
	uint8_t response = readRegister8(_i2caddr, MAX30102_INTSTAT2);
    if ((response & MAX30102_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
  //TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readRegister8(_i2caddr, MAX30102_DIETEMPINT);
  uint8_t tempFrac = readRegister8(_i2caddr, MAX30102_DIETEMPFRAC); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float readTemperatureF() {
  float temp = readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}


//Setup the sensor
//The MAX30102 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30102 sensor
void setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange) {
  
  softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) setFIFOAverage(MAX30102_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) setFIFOAverage(MAX30102_SAMPLEAVG_2);
  else if (sampleAverage == 4) setFIFOAverage(MAX30102_SAMPLEAVG_4);
  else if (sampleAverage == 8) setFIFOAverage(MAX30102_SAMPLEAVG_8);
  else if (sampleAverage == 16) setFIFOAverage(MAX30102_SAMPLEAVG_16);
  else if (sampleAverage == 32) setFIFOAverage(MAX30102_SAMPLEAVG_32);
  else setFIFOAverage(MAX30102_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) setLEDMode(MAX30102_MODE_MULTILED); //Watch all three LED channels
  else if (ledMode == 2) setLEDMode(MAX30102_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX30102_MODE_REDONLY); //Red only
  if(ledMode > 2) ledMode = 2;
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) setADCRange(MAX30102_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30102_ADCRANGE_2048);

  if (sampleRate < 100) setSampleRate(MAX30102_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) setSampleRate(MAX30102_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX30102_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX30102_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX30102_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX30102_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX30102_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX30102_SAMPLERATE_3200);
  else setSampleRate(MAX30102_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) setPulseWidth(MAX30102_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) setPulseWidth(MAX30102_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) setPulseWidth(MAX30102_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) setPulseWidth(MAX30102_PULSEWIDTH_411); //18 bit resolution
  else setPulseWidth(MAX30102_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}

//
// Data Collection
//

//Tell caller how many samples are available
uint8_t available(void)
{

  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent red value
uint32_t getRed(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.red[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t getGreen(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.green[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the next Red value in the FIFO
uint32_t getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

//Report the next IR value in the FIFO
uint32_t getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}

//Report the next Green value in the FIFO
uint32_t getFIFOGreen(void)
{
  return (sense.green[sense.tail]);
}

//Advance the tail
void nextSample(void)
{
  if(available()) //Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition
  }
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    // _i2cPort->beginTransmission(MAX30102_ADDRESS);
    // _i2cPort->write(MAX30102_FIFODATA);
    // _i2cPort->endTransmission();
    i2c_write_blocking(_i2cPort, _i2caddr, &MAX30102_FIFODATA, 1, true);

    //We may need to read as many as 192 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      i2c_read_burst_blocking(_i2cPort, _i2caddr, readByteStream, toGet);
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = readByteStream[0];
        temp[1] = readByteStream[1];
        temp[0] = readByteStream[2];

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));
	      tempLong &= 0x3FFFF; //Zero out all but 18 bits
        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = readByteStream[3];
          temp[1] = readByteStream[4];
          temp[0] = readByteStream[5];

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));
          tempLong &= 0x3FFFF; //Zero out all but 18 bits
          sense.IR[sense.head] = tempLong;
        }

      // *****Currently unused for MAX30102*****

      //   if (activeLEDs > 2)
      //   {
      //     //Burst read three more bytes - Green
      //     temp[3] = 0;
      //     temp[2] = _i2cPort->read();
      //     temp[1] = _i2cPort->read();
      //     temp[0] = _i2cPort->read();

      //     //Convert array to long
      //     memcpy(&tempLong, temp, sizeof(tempLong));

		  // tempLong &= 0x3FFFF; //Zero out all but 18 bits

      //     sense.green[sense.head] = tempLong;
      //   }
      
      // ***************************************

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
bool safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();
  
  while(1)
  {
	if(millis() - markTime > maxTimeToCheck) return(false);

	if(check() == true) //We found new data!
	  return(true);

	delay(1);
  }
}


//
// Device ID and Revision
//
uint8_t readPartID() {
  
    return readRegister8(_i2caddr, MAX30102_PARTID);
}

//
// Low-level I2C Communication
//

//Given a register, read it, mask it, and then set the thing
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(_i2caddr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(_i2caddr, reg, originalContents | thing);
}

uint8_t readRegister8(uint8_t address, uint8_t reg) {
    i2c_write_blocking(_i2cPort, _i2caddr, &reg, 1, true);
    i2c_read_blocking(_i2cPort, _i2caddr, &readByte, 1, false);
    return readByte;
}

void writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
    writePacket[0]=reg;
    writePacket[1]=value;
    i2c_write_blocking(_i2cPort, _i2caddr, writePacket, 2, false);
}

void readLoop(uint8_t address, uint8_t reg){
  i2c_write_blocking(_i2cPort, _i2caddr, &reg, 1, true);
  while(true){
    i2c_read_blocking(_i2cPort, _i2caddr, &readByte, 1, true);
    printf("%d\n", readByte);
  }
}

//
// End Low-level I2C Communication
//

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength = 100; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

int main()
{
    int ack;
    
    size_t nBytes;

    //Debug functions init (USB serial & onboard LED)
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    

    // // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    

    //I2C Init
    i2c_init(I2C_PORT, 100*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Pulse ox init
    max30102_init(I2C_PORT, I2C_SPEED_FAST, MAX30102_ADDRESS);

    

    // Pulse Ox setup initial values
    byte ledBrightness = 80; //Options: 0=Off to 255=50mA
    byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange = 16384; //Options: 2048, 4096, 8192, 16384

    setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
   
    while (true) {
      
      // for(byte i = 0; i < bufferLength; i++){

      //   while(available() == false)
      //     check();
        

      //   redBuffer[i] = getRed();
      //   irBuffer[i] = getIR();
      //   nextSample();

      //   printf("red=%d", redBuffer[i]);
      //   printf(", ");
      //   printf("ir=%d", irBuffer[i]);
      //   printf("\n");


      // }

      readLoop(MAX30102_ADDRESS, MAX30102_FIFODATA);
      
        
    }
}
