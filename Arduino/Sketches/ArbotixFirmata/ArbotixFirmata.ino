/*
 * ArbotixFirmata is a generic protocol for communicating with microcontrollers
 * from software on a host computer. It is intended to work with
 * any host computer software package.
 *
 * To download a host software package, please clink on the following link
 * to open the download page in your default browser.
 *
 * http://firmata.org/wiki/Download
 */

/*
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2011 Jeff Hoefs.  All rights reserved.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
 
  See file LICENSE.txt for further informations on licensing terms.

  formatted using the GNU C formatting and indenting
*/

/* 
 * TODO: use Program Control to load stored profiles from EEPROM
 */

#include <Servo.h>
#include <Wire.h>
#include <Firmata.h>
#include <SoftwareSerial.h>
//#include "CommanderSS.h"
#include <ax12.h>

// move the following defines to Firmata.h?
#define I2C_WRITE B00000000
#define I2C_READ B00001000
#define I2C_READ_CONTINUOUSLY B00010000
#define I2C_STOP_READING B00011000
#define I2C_READ_WRITE_MODE_MASK B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000

#define MAX_QUERIES 8
#define MINIMUM_SAMPLING_INTERVAL 10

#define REGISTER_NOT_SPECIFIED -1

#define SYSEX_DYNAMIXEL_KEY_SERVO_DATA      0x68 // Data packet of key (pos, speed) Dynamixel data.
#define SYSEX_DYNAMIXEL_ALL_SERVO_DATA      0x67 // Data packet of all (pos, speed, load, voltage, temp) Dynamixel data.
#define SYSEX_DYNAMIXEL_CONFIG		    0x66 // Data packet to configure firmata to listen for dynamixel data.
#define SYSEX_DYNAMIXEL_SYNCH_MOVE_START    0x65 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_SYNCH_MOVE_ADD	    0x64 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_SYNCH_MOVE_EXECUTE  0x63 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_MOVE		    0x62 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_STOP		    0x61 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_TRANSMIT_ERROR	    0x60 // Data packet to for when there is a transmission error detected on the arbotix side.
#define SYSEX_DYNAMIXEL_SET_REGISTER        0x59 // Data packet to set a specific register in a Dynamixel servo.
#define SYSEX_DYNAMIXEL_GET_REGISTER	    0x58 // Data packet to get a specific register in a Dynamixel servo.
#define SYSEX_DYNAMIXEL_CONFIGURE_SERVO	    0x57 // Data packet to configure key motor params like cw and ccw limits.
#define SYSEX_DYNAMIXEL_STOPPED  	    0x56 // Data packet to configure reporting of when a servo stops moving.
#define SYSEX_COMMANDER_DATA		    0x55 // Data packet with commander remote control buttons pressed.


#define DYNAMIXEL_TOTAL_SERVOS 30
#define DYNAMIXEL_RX_PIN 10	 
#define DYNAMIXEL_TX_PIN 11

//#define ENABLE_COMMANDER 1
#define COMMANDER_RX_PIN 2	 
#define COMMANDER_TX_PIN 3

//#ifdef ENABLE_COMMANDER
  //If defined then it setups an additional serial debug port to use.
  #define DEBUG_SERIAL 1
//#endif

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 19;          // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  byte reg;
  byte bytes;
};

/* for i2c read continuous more */
i2c_device_info query[MAX_QUERIES];

byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;  // default delay time between i2c read request and Wire.requestFrom()

Servo servos[MAX_SERVOS];

byte customSysExLength = 0;
byte aryCustomSysEx[64];

struct dynamixelData {
  byte servo;
  int pos;
  int speed;
};

struct dynamixelReadData {
  byte data[8];
};

//This controls how often the full set of motor data is sent back to the computer.
byte reportDynDataCount = 20;

//This keeps track of how many iterations have occurred since we last sent back the full data set.
byte reportDynDataIdx = 0;

//This keeps track of which servos are setup to report their data back to the computer.
byte reportDynServos[DYNAMIXEL_TOTAL_SERVOS];       // 1 = report this servo, 0 = silence

//This keeps track of which servos need to be monitored for movement.
byte reportDynServosMove[DYNAMIXEL_TOTAL_SERVOS];       // 0 = do not monitor, 1 = report when movement stops 

//The total number of servos in the current synch command that is being built
byte totalSynchServos=0;

//Data used to write out a synch move command
dynamixelData synchMoveData[DYNAMIXEL_TOTAL_SERVOS];

//Stores the last read value of the dynamixel servo.
//If the servo value has not changed since the last time then do not bother
//sending the data back to the computer
dynamixelReadData reportDynData[DYNAMIXEL_TOTAL_SERVOS];

#ifdef DEBUG_SERIAL
  SoftwareSerial debugSerial = SoftwareSerial(COMMANDER_RX_PIN, COMMANDER_TX_PIN); // RX, TX
#endif

#ifdef ENABLE_COMMANDER
  CommanderData commanderData;
  CommanderSS command = CommanderSS(&debugSerial);
#endif

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void readAndReportData(byte address, int theRegister, byte numBytes) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()  
  if (theRegister != REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    #if ARDUINO >= 100
    Wire.write((byte)theRegister);
    #else
    Wire.send((byte)theRegister);
    #endif
    Wire.endTransmission();
    delayMicroseconds(i2cReadDelayTime);  // delay is necessary for some devices such as WiiNunchuck
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if(numBytes == Wire.available()) {
    i2cRxData[0] = address;
    i2cRxData[1] = theRegister;
    for (int i = 0; i < numBytes; i++) {
      #if ARDUINO >= 100
      i2cRxData[2 + i] = Wire.read();
      #else
      i2cRxData[2 + i] = Wire.receive();
      #endif
    }
  }
  else {
    if(numBytes > Wire.available()) {
      Firmata.sendString("I2C Read Error: Too many bytes received");
    } else {
      Firmata.sendString("I2C Read Error: Too few bytes received"); 
    }
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if(forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  //Do not attempt to set the pin mode for dynamixel IO pins.
  if(pin == DYNAMIXEL_RX_PIN || pin == DYNAMIXEL_TX_PIN)
    return;

#ifdef ENABLE_COMMANDER || DEBUG_SERIAL
  //if the debug serial is on then skip config of these pins also.
  if(pin == COMMANDER_RX_PIN || pin == COMMANDER_TX_PIN)
    return;
#endif

  if (pinConfig[pin] == I2C && isI2CEnabled && mode != I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_SERVO(pin) && mode != SERVO && servos[PIN_TO_SERVO(pin)].attached()) {
    servos[PIN_TO_SERVO(pin)].detach();
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) { // || mode == INPUT_PULLUP
      portConfigInputs[pin/8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin/8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch(mode) {
  case ANALOG:
    if (IS_PIN_ANALOG(pin)) {
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      }
      pinConfig[pin] = ANALOG;
    }
    break;
  case INPUT:
    if (IS_PIN_DIGITAL(pin)) {
      pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      pinConfig[pin] = INPUT;
    }
    break;
  case OUTPUT:
    if (IS_PIN_DIGITAL(pin)) {
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
      pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
      pinConfig[pin] = OUTPUT;
    }
    break;
  case PWM:
    if (IS_PIN_PWM(pin)) {
      pinMode(PIN_TO_PWM(pin), OUTPUT);
      analogWrite(PIN_TO_PWM(pin), 0);
      pinConfig[pin] = PWM;
    }
    break;
  case SERVO:
    if (IS_PIN_SERVO(pin)) {
      pinConfig[pin] = SERVO;
      if (!servos[PIN_TO_SERVO(pin)].attached()) {
          servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin));
      }
    }
    break;
  case I2C:
    if (IS_PIN_I2C(pin)) {
      // mark the pin as i2c
      // the user must call I2C_CONFIG to enable I2C for a device
      pinConfig[pin] = I2C;
    }
    break;
/*  case INPUT_PULLUP:
    if (IS_PIN_DIGITAL(pin)) {
      pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
      digitalWrite(PIN_TO_DIGITAL(pin), HIGH); // enable internal pull-ups
      pinConfig[pin] = INPUT_PULLUP;
    }
    break;*/
  default:
    Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch(pinConfig[pin]) {
    case SERVO:
      if (IS_PIN_SERVO(pin))
        servos[PIN_TO_SERVO(pin)].write(value);
        pinState[pin] = value;
      break;
    case PWM:
      if (IS_PIN_PWM(pin))
        analogWrite(PIN_TO_PWM(pin), value);
        pinState[pin] = value;
      break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask=1, pinWriteMask=0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port*8+8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin=port*8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if(value == 0) {
      analogInputsToReport = analogInputsToReport &~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

void dynamixelSynchMoveSetup(byte servo, int pos, int speed) {
  
  //Check to see if this servo has already been added.
  //If it has then set it again but do not add a new one.
  for(int i=0; i<totalSynchServos; i++) {
    if(synchMoveData[i].servo == servo) {
      synchMoveData[totalSynchServos].servo = servo;
      synchMoveData[totalSynchServos].pos = pos;
      synchMoveData[totalSynchServos].speed = speed;
      return;
    }    
  }

  synchMoveData[totalSynchServos].servo = servo;
  synchMoveData[totalSynchServos].pos = pos;
  synchMoveData[totalSynchServos].speed = speed;
  totalSynchServos++;
}

void dynamixelSynchMoveExecute() {
  
  ax12StartSyncWrite();
  for(int i=0; i<totalSynchServos; i++) {
    ax12AddServoToSync(synchMoveData[i].servo, synchMoveData[i].pos, synchMoveData[i].speed);
  }
  ax12WriteSyncData(); 

  //Reset the synch servo list to get ready for the next move command
  totalSynchServos = 0;
}

void sendDynamixelTransmitError(byte command, byte servo, byte recChecksum, byte calcChecksum) {
  customSysExLength = 3;
  aryCustomSysEx[0] = command;
  aryCustomSysEx[1] = servo;
  int checksum = (~(command + servo)) & 0xFF;
  aryCustomSysEx[2] = checksum;

#ifdef DEBUG_SERIAL
  debugSerial.println("Sending transmit error");  
  debugSerial.print("cmd: "); debugSerial.print(command);
  debugSerial.print(", servo: "); debugSerial.print(servo);
  debugSerial.print(", recChecksum: "); debugSerial.print(recChecksum);
  debugSerial.print(", calcChecksum: "); debugSerial.print(calcChecksum);
  debugSerial.print(", checksum: "); debugSerial.print(checksum);
  debugSerial.print ("\n");
#endif

  // send Dynamixel error data
  Firmata.sendSysex(SYSEX_DYNAMIXEL_TRANSMIT_ERROR, customSysExLength, aryCustomSysEx);
}

void sendDynamixelRegister(byte servo, byte reg, byte length, int value) {
  customSysExLength = 5;
  aryCustomSysEx[0] = servo;
  aryCustomSysEx[1] = reg;
  aryCustomSysEx[2] = ax12GetLowByte(value);
  aryCustomSysEx[3] = ax12GetHighByte(value);
  int checksum = (~(servo + reg + value)) & 0xFF;
  aryCustomSysEx[4] = checksum;

#ifdef DEBUG_SERIAL
  debugSerial.println("Sending dynamixel register");  
  debugSerial.print("servo: "); debugSerial.print(servo);
  debugSerial.print(", reg: "); debugSerial.print(reg);
  debugSerial.print(", length: "); debugSerial.print(length);
  debugSerial.print(", value: "); debugSerial.print(value);
  debugSerial.print(", checksum: "); debugSerial.print(checksum);
  debugSerial.print ("\n");
#endif

  // send Dynamixel error data
  Firmata.sendSysex(SYSEX_DYNAMIXEL_GET_REGISTER, customSysExLength, aryCustomSysEx);
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte slaveRegister;
  byte data;
  unsigned int delayTime; 
  
  switch(command) {
  case REPORT_FIRMWARE:
#ifdef DEBUG_SERIAL
      debugSerial.println("*** Sending firmware version");  
#endif
  
    Firmata.printFirmwareVersion();
    break;
  case I2C_REQUEST:
    mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
    if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
      Firmata.sendString("10-bit addressing mode is not yet supported");
      return;
    }
    else {
      slaveAddress = argv[0];
    }

    switch(mode) {
    case I2C_WRITE:
      Wire.beginTransmission(slaveAddress);
      for (byte i = 2; i < argc; i += 2) {
        data = argv[i] + (argv[i + 1] << 7);
        #if ARDUINO >= 100
        Wire.write(data);
        #else
        Wire.send(data);
        #endif
      }
      Wire.endTransmission();
      delayMicroseconds(70);
      break;
    case I2C_READ:
      if (argc == 6) {
        // a slave register is specified
        slaveRegister = argv[2] + (argv[3] << 7);
        data = argv[4] + (argv[5] << 7);  // bytes to read
        readAndReportData(slaveAddress, (int)slaveRegister, data);
      }
      else {
        // a slave register is NOT specified
        data = argv[2] + (argv[3] << 7);  // bytes to read
        readAndReportData(slaveAddress, (int)REGISTER_NOT_SPECIFIED, data);
      }
      break;
    case I2C_READ_CONTINUOUSLY:
      if ((queryIndex + 1) >= MAX_QUERIES) {
        // too many queries, just ignore
        Firmata.sendString("too many queries");
        break;
      }
      queryIndex++;
      query[queryIndex].addr = slaveAddress;
      query[queryIndex].reg = argv[2] + (argv[3] << 7);
      query[queryIndex].bytes = argv[4] + (argv[5] << 7);
      break;
    case I2C_STOP_READING:
	  byte queryIndexToSkip;      
      // if read continuous mode is enabled for only 1 i2c device, disable
      // read continuous reporting for that device
      if (queryIndex <= 0) {
        queryIndex = -1;        
      } else {
        // if read continuous mode is enabled for multiple devices,
        // determine which device to stop reading and remove it's data from
        // the array, shifiting other array data to fill the space
        for (byte i = 0; i < queryIndex + 1; i++) {
          if (query[i].addr = slaveAddress) {
            queryIndexToSkip = i;
            break;
          }
        }
        
        for (byte i = queryIndexToSkip; i<queryIndex + 1; i++) {
          if (i < MAX_QUERIES) {
            query[i].addr = query[i+1].addr;
            query[i].reg = query[i+1].addr;
            query[i].bytes = query[i+1].bytes; 
          }
        }
        queryIndex--;
      }
      break;
    default:
      break;
    }
    break;
  case I2C_CONFIG:
    delayTime = (argv[0] + (argv[1] << 7));

    if(delayTime > 0) {
      i2cReadDelayTime = delayTime;
    }

    if (!isI2CEnabled) {
      enableI2CPins();
    }
    
    break;
  case SERVO_CONFIG:
    if(argc > 4) {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte pin = argv[0];
      int minPulse = argv[1] + (argv[2] << 7);
      int maxPulse = argv[3] + (argv[4] << 7);

      if (IS_PIN_SERVO(pin)) {
        if (servos[PIN_TO_SERVO(pin)].attached())
          servos[PIN_TO_SERVO(pin)].detach();
        servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
        setPinModeCallback(pin, SERVO);
      }
    }
    break;
  case SAMPLING_INTERVAL:
    if (argc > 1) {
      samplingInterval = argv[0] + (argv[1] << 7);
      if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
        samplingInterval = MINIMUM_SAMPLING_INTERVAL;
      }      
    } else {
      //Firmata.sendString("Not enough data");
    }
    break;
  case EXTENDED_ANALOG:
    if (argc > 1) {
      int val = argv[1];
      if (argc > 2) val |= (argv[2] << 7);
      if (argc > 3) val |= (argv[3] << 14);
      analogWriteCallback(argv[0], val);
    }
    break;
  case CAPABILITY_QUERY:
    Serial.write(START_SYSEX);
    Serial.write(CAPABILITY_RESPONSE);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_DIGITAL(pin)) {
        Serial.write((byte)INPUT);
        Serial.write(1);
        Serial.write((byte)OUTPUT);
        Serial.write(1);
      }
      if (IS_PIN_ANALOG(pin)) {
        Serial.write(ANALOG);
        Serial.write(10);
      }
      if (IS_PIN_PWM(pin)) {
        Serial.write(PWM);
        Serial.write(8);
      }
      if (IS_PIN_SERVO(pin)) {
        Serial.write(SERVO);
        Serial.write(14);
      }
      if (IS_PIN_I2C(pin)) {
        Serial.write(I2C);
        Serial.write(1);  // to do: determine appropriate value 
      }
      Serial.write(127);
    }
    Serial.write(END_SYSEX);
    break;
  case PIN_STATE_QUERY:
    if (argc > 0) {
      byte pin=argv[0];
      Serial.write(START_SYSEX);
      Serial.write(PIN_STATE_RESPONSE);
      Serial.write(pin);
      if (pin < TOTAL_PINS) {
        Serial.write((byte)pinConfig[pin]);
	Serial.write((byte)pinState[pin] & 0x7F);
	if (pinState[pin] & 0xFF80) Serial.write((byte)(pinState[pin] >> 7) & 0x7F);
	if (pinState[pin] & 0xC000) Serial.write((byte)(pinState[pin] >> 14) & 0x7F);
      }
      Serial.write(END_SYSEX);
    }
    break;
  case ANALOG_MAPPING_QUERY:
    Serial.write(START_SYSEX);
    Serial.write(ANALOG_MAPPING_RESPONSE);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      Serial.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
    }
    Serial.write(END_SYSEX);
    break;
  case SYSEX_DYNAMIXEL_CONFIG: {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte servo = argv[0] + (argv[1] << 7);
      byte report = argv[2];
      
      reportDynServos[servo] = report;

#ifdef DEBUG_SERIAL
      debugSerial.print("Recieved Dynamixel Config");  
      debugSerial.print(", Servo: "); debugSerial.print(servo);
      debugSerial.print(", Report: "); debugSerial.print(report);
      debugSerial.print ("\n");
#endif
    }
    break;
  case SYSEX_DYNAMIXEL_SYNCH_MOVE_START: {
#ifdef DEBUG_SERIAL
      //debugSerial.println("Recieved start Dynamixel synch move");
#endif
      totalSynchServos = 0;
    }
    break;
  case SYSEX_DYNAMIXEL_SYNCH_MOVE_ADD: {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte servo = argv[0] + (argv[1] << 7);
      byte pos0 = argv[2] + (argv[3] << 7);
      byte pos1 = argv[4] + (argv[5] << 7);
      int pos = ax12MakeWord(pos0, pos1);
      byte speed0 = argv[6] + (argv[7] << 7);
      byte speed1 = argv[8] + (argv[9] << 7);
      int speed = ax12MakeWord(speed0, speed1);
      byte recChecksum = argv[10] + (argv[11] << 7);
      int checksum = (~(servo + pos0 + pos1 + speed0 + speed1)) & 0xFF;
      
      if(recChecksum == checksum)  
        dynamixelSynchMoveSetup(servo, pos, speed);
      else
        sendDynamixelTransmitError(SYSEX_DYNAMIXEL_SYNCH_MOVE_ADD, servo, recChecksum, checksum);
      
#ifdef DEBUG_SERIAL
      //debugSerial.println("Recieved Dynamixel synch move add");  
      //debugSerial.print(",count: "); debugSerial.print(totalSynchServos);
      //debugSerial.print(",servo: "); debugSerial.print(servo);
      //debugSerial.print(", pos: "); debugSerial.print(pos);
      //debugSerial.print(", speed: "); debugSerial.print(speed);
      //debugSerial.print(", rec checksum: "); debugSerial.print(recChecksum);
      //debugSerial.print(", checksum: "); debugSerial.print(checksum);
      //debugSerial.print ("\n");
#endif
    }
    break;
  case SYSEX_DYNAMIXEL_SYNCH_MOVE_EXECUTE: {
#ifdef DEBUG_SERIAL
      //debugSerial.println("Recieved execute Dynamixel synch move");
#endif
      dynamixelSynchMoveExecute();
    }
    break;
  case SYSEX_DYNAMIXEL_MOVE: {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte servo = argv[0] + (argv[1] << 7);
      byte pos0 = argv[2] + (argv[3] << 7);
      byte pos1 = argv[4] + (argv[5] << 7);
      int pos = ax12MakeWord(pos0, pos1);
      byte speed0 = argv[6] + (argv[7] << 7);
      byte speed1 = argv[8] + (argv[9] << 7);
      int speed = ax12MakeWord(speed0, speed1);
      byte recChecksum = argv[10] + (argv[11] << 7);
      int checksum = (~(servo + pos0 + pos1 + speed0 + speed1)) & 0xFF;

      if(recChecksum == checksum)  
      {
        SetSpeed(servo, speed);
        delay(10);
        SetPosition(servo, pos);
      }
      else
        sendDynamixelTransmitError(SYSEX_DYNAMIXEL_MOVE, servo, recChecksum, checksum);
    
#ifdef DEBUG_SERIAL
      debugSerial.print("Recieved Dynamixel move ");  
      debugSerial.print(", servo: "); debugSerial.print(servo);
      debugSerial.print(", pos: "); debugSerial.print(pos);
      debugSerial.print(", speed: "); debugSerial.print(speed);
      debugSerial.print(", rec checksum: "); debugSerial.print(recChecksum);
      debugSerial.print(", checksum: "); debugSerial.print(checksum);
      debugSerial.print ("\n");
#endif
    }
    break;
  case SYSEX_DYNAMIXEL_STOP: {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte servo = argv[0] + (argv[1] << 7);
      byte recChecksum = argv[2] + (argv[3] << 7);
      int checksum = (~(servo)) & 0xFF;
      int pos = 0;
      
      if(recChecksum == checksum)  
      {
        //First set speed to slowest setting.
        SetSpeed(servo, 1);
        delay(10);
        
        //Now get the servo position
        pos = GetPosition(servo);
        
        //And set the goal position to that value.
        SetPosition(servo, pos);
      }
      else
        sendDynamixelTransmitError(SYSEX_DYNAMIXEL_STOP, servo, recChecksum, checksum);
    
#ifdef DEBUG_SERIAL
      debugSerial.print("Recieved Dynamixel Stop ");  
      debugSerial.print(", servo: "); debugSerial.print(servo);
      debugSerial.print(", pos: "); debugSerial.print(pos);
      debugSerial.print(", rec checksum: "); debugSerial.print(recChecksum);
      debugSerial.print(", checksum: "); debugSerial.print(checksum);
      debugSerial.print ("\n");
#endif
    }
    break;
  case SYSEX_DYNAMIXEL_SET_REGISTER: {
      byte servo = argv[0] + (argv[1] << 7);
      byte reg = argv[2] + (argv[3] << 7);
      byte length = argv[4] + (argv[5] << 7);
      byte value0 = argv[6] + (argv[7] << 7);
      byte value1 = argv[8] + (argv[9] << 7);

      int value = ax12MakeWord(value0, value1);
        
      byte recChecksum = argv[10] + (argv[11] << 7);
      int checksum = (~(servo + reg + length + value0 + value1)) & 0xFF;

      if(recChecksum == checksum) {
        if(length == 1) 
          ax12SetRegister(servo, reg, value);
         else
          ax12SetRegister2(servo, reg, value);
      }
      else
        sendDynamixelTransmitError(SYSEX_DYNAMIXEL_SET_REGISTER, servo, recChecksum, checksum);
    
#ifdef DEBUG_SERIAL
      debugSerial.print("Set Dynamixel register");
      debugSerial.print(", servo: "); debugSerial.print(servo);
      debugSerial.print(", reg: "); debugSerial.print(reg);
      debugSerial.print(", length: "); debugSerial.print(length);
      debugSerial.print(", value: "); debugSerial.print(value);
      debugSerial.print(", rec checksum: "); debugSerial.print(recChecksum);
      debugSerial.print(", checksum: "); debugSerial.print(checksum);
      debugSerial.print ("\n");
#endif
    }
    break;
  case SYSEX_DYNAMIXEL_GET_REGISTER: {
      byte servo = argv[0] + (argv[1] << 7);
      byte reg = argv[2] + (argv[3] << 7);
      byte length = argv[4] + (argv[5] << 7);
      byte recChecksum = argv[6] + (argv[7] << 7);
      int checksum = (~(servo + reg + length)) & 0xFF;

      if(recChecksum == checksum) {
        //ax12SetRegister2(servo, reg, 310);
        //delay(20);
        int regVal = ax12GetRegister(servo, reg, length);
#ifdef DEBUG_SERIAL
      debugSerial.print("RegVal: "); debugSerial.println(regVal);
#endif        
        sendDynamixelRegister(servo, reg, length, regVal);
      }
      else
        sendDynamixelTransmitError(SYSEX_DYNAMIXEL_GET_REGISTER, servo, recChecksum, checksum);
    }
    break;
  case SYSEX_DYNAMIXEL_CONFIGURE_SERVO: {
      byte servo = argv[0] + (argv[1] << 7);
      byte cwlimit0 = argv[2] + (argv[3] << 7);
      byte cwlimit1 = argv[4] + (argv[5] << 7);
      byte ccwlimit0 = argv[6] + (argv[7] << 7);
      byte ccwlimit1 = argv[8] + (argv[9] << 7);
      byte maxtorque0 = argv[10] + (argv[11] << 7);
      byte maxtorque1 = argv[12] + (argv[13] << 7);
      byte delaytime = argv[14] + (argv[15] << 7);
      byte recChecksum = argv[16] + (argv[17] << 7);
      int checksum = (~(servo + cwlimit0 + cwlimit1 + ccwlimit0 + ccwlimit1 +
                      maxtorque0 + maxtorque1 + delaytime)) & 0xFF;

      int cwlimit = ax12MakeWord(cwlimit0, cwlimit1);
      int ccwlimit = ax12MakeWord(ccwlimit0, ccwlimit1);
      int maxtorque = ax12MakeWord(maxtorque0, maxtorque1);

      if(recChecksum == checksum) {
        if(GetCWLimit(servo) != cwlimit)
        {
          SetCWLimit(servo, cwlimit);
          delay(10);
        }

        if(GetCCWLimit(servo) != ccwlimit)
        {
          SetCCWLimit(servo, ccwlimit);
          delay(10);
        }

        if(GetMaxTorque(servo) != maxtorque)
        {
          SetMaxTorque(servo, maxtorque);
          delay(10);
        }

        if(GetReturnDelayTime(servo) != delaytime)
        {
          SetReturnDelayTime(servo, delaytime);
          delay(10);
        }
      }
      else
        sendDynamixelTransmitError(SYSEX_DYNAMIXEL_CONFIGURE_SERVO, servo, recChecksum, checksum);
    
#ifdef DEBUG_SERIAL
      debugSerial.print("Dynamixel config servo");
      debugSerial.print(", servo: "); debugSerial.print(servo);
      debugSerial.print(", cwlimit: "); debugSerial.print(cwlimit);
      debugSerial.print(", ccwlimit: "); debugSerial.print(ccwlimit);
      debugSerial.print(", maxtorque: "); debugSerial.print(maxtorque);
      debugSerial.print(", delaytime: "); debugSerial.print(delaytime);
      debugSerial.print(", rec checksum: "); debugSerial.print(recChecksum);
      debugSerial.print(", checksum: "); debugSerial.print(checksum);
      debugSerial.print ("\n");
#endif
    }
    break;
  case SYSEX_DYNAMIXEL_STOPPED: {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte servo = argv[0] + (argv[1] << 7);
      byte report = argv[2];
      
      reportDynServosMove[servo] = report;

#ifdef DEBUG_SERIAL
      debugSerial.print("Recieved Dynamixel Stopped ");  
      debugSerial.print(", Servo: "); debugSerial.print(servo);
      debugSerial.print(", Report: "); debugSerial.print(report);
      debugSerial.print ("\n");
#endif
    }
    break;
  }
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing 
  // Arduino.h to get SCL and SDA pins
  for (i=0; i < TOTAL_PINS; i++) {
    if(IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, I2C);
    } 
  }
   
  isI2CEnabled = true; 
  
  // is there enough time before the first I2C request to call this here?
  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
    isI2CEnabled = false;
    // disable read continuous mode for all devices
    queryIndex = -1;
    // uncomment the following if or when the end() method is added to Wire library
    // Wire.end();
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void systemResetCallback()
{
  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default
  if (isI2CEnabled) {
  	disableI2CPins();
  }
    
  for (byte i=0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;      // by default, reporting off
    portConfigInputs[i] = 0;	// until activated
    previousPINs[i] = 0;
  }
  
  // pins with analog capability default to analog input
  // otherwise, pins default to digital output
  for (byte i=0; i < TOTAL_PINS; i++) {
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  //clear out the dynamixel servos we need to report
  for (byte i=0; i<DYNAMIXEL_TOTAL_SERVOS; i++)
  {
    reportDynServos[i] = 0;
    reportDynServosMove[i] = 0;
    
    for(byte j=0; j<8; j++)
      reportDynData[i].data[j] = -1;
  }

#ifdef DEBUG_SERIAL
  debugSerial.println("Called reset: ");
#endif

#ifdef ENABLE_COMMANDER
  //Setup command data with bad values at the start so as soon as we get an incoming packet it will 
  //not match the default positions and need to send the first packet.
  commanderData.walkV = -250;
  commanderData.walkH = -250;
  commanderData.lookV = -250;
  commanderData.lookH = -250;
  commanderData.buttons = -250;
#endif

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
}

void sendDynamixelAllData(byte servo)
{
  customSysExLength = 10;
  aryCustomSysEx[0] = servo;
  
  //Read all the data (pos, speed, load, volt, temp) from the servo
  if(ax12GetAllMovementRegisterBytes(servo, &aryCustomSysEx[1]) > 0) {

    //Only send the data to the computer if the dynamixel data does not match what we sent last time.
    if(memcmp(reportDynData[servo].data, &aryCustomSysEx[1], 8) != 0)
    {    
      int pos = aryCustomSysEx[1] + (aryCustomSysEx[2] << 8);
      int speed = aryCustomSysEx[3] + (aryCustomSysEx[4] << 8);
      int load = aryCustomSysEx[5] + (aryCustomSysEx[6] << 8);

      int checksum = (~(aryCustomSysEx[0] + pos + speed + load + aryCustomSysEx[7] + aryCustomSysEx[8])) & 0xFF;
      aryCustomSysEx[9] = checksum;
      
#ifdef DEBUG_SERIAL
      //debugSerial.print("All  Servo: "); debugSerial.print(servo);
      //debugSerial.print(", Pos: "); debugSerial.print(pos); 
      //debugSerial.print(", speed: "); debugSerial.print(speed); 
      //debugSerial.print(", load: "); debugSerial.print(load); 
      //debugSerial.print(", volt: "); debugSerial.print(aryCustomSysEx[7]); 
      //debugSerial.print(", temp: "); debugSerial.print(aryCustomSysEx[8]); 
      //debugSerial.print(", checksum: "); debugSerial.print(checksum); 
      //debugSerial.print ("\n");
#endif
    
      // send Dynamixel data
      Firmata.sendSysex(SYSEX_DYNAMIXEL_ALL_SERVO_DATA, customSysExLength, aryCustomSysEx);
      
      //Copy the data we just sent into our history buffer for the next time.
      memcpy(reportDynData[servo].data, &aryCustomSysEx[1], 8);      
    }
  }
}

void sendDynamixelKeyData(byte servo)
{
  customSysExLength = 6;
  aryCustomSysEx[0] = servo;
  
  //Read the key data (pos, speed) from the servo
  if(ax12GetKeyMovementRegisterBytes(servo, &aryCustomSysEx[1]) > 0) {

    //Only send the data to the computer if the dynamixel data does not match what we sent last time.
    if(memcmp(reportDynData[servo].data, &aryCustomSysEx[1], 4) != 0)
    {    
      int pos = aryCustomSysEx[1] + (aryCustomSysEx[2] << 8);
      int speed = aryCustomSysEx[3] + (aryCustomSysEx[4] << 8);
      
      int checksum = (~(aryCustomSysEx[0] + pos + speed)) & 0xFF;
      aryCustomSysEx[5] = checksum;
      
#ifdef DEBUG_SERIAL
      //debugSerial.print("Key  Servo: "); debugSerial.print(servo); 
      //debugSerial.print(", pos: "); debugSerial.print(pos); 
      //debugSerial.print(", speed: "); debugSerial.print(speed); 
      //debugSerial.print(", checksum: "); debugSerial.print(checksum); 
      //debugSerial.print ("\n");
#endif

      // send Dynamixel data
      Firmata.sendSysex(SYSEX_DYNAMIXEL_KEY_SERVO_DATA, customSysExLength, aryCustomSysEx);

      //Copy the data we just sent into our history buffer for the next time.
      memcpy(reportDynData[servo].data, &aryCustomSysEx[1], 4);      
    }
  }
}

void sendDynamixelStopped(byte servo)
{
  customSysExLength = 2;
  aryCustomSysEx[0] = servo;
  
  int checksum = (~(servo)) & 0xFF;
  aryCustomSysEx[1] = checksum;
      
#ifdef DEBUG_SERIAL
      debugSerial.print("SendDynamixelStopped  Servo: "); debugSerial.println(servo); 
#endif

  // send Dynamixel data
  Firmata.sendSysex(SYSEX_DYNAMIXEL_STOPPED, customSysExLength, aryCustomSysEx);
}

//Go through each of the servo motors that the user has configured to monitor and 
//read them and send back data on them.
void checkDynamixelMotors()
{
  for(byte servo=0; servo<DYNAMIXEL_TOTAL_SERVOS; servo++) {
    if(reportDynServos[servo]) {
      if(reportDynDataIdx == reportDynDataCount)
        sendDynamixelAllData(servo);
      else
        sendDynamixelKeyData(servo);
    }
    
    if(reportDynServosMove[servo]) {
      if(!IsMoving(servo)) {
        reportDynServosMove[servo] = false;
        sendDynamixelStopped(servo);        
      }
    }
  }
  
  if(reportDynDataIdx == reportDynDataCount)
    reportDynDataIdx = 0;      
  else
    reportDynDataIdx++;
}

#ifdef ENABLE_COMMANDER

void checkCommander() {
  
  if(command.ReadMsgs() > 0) {
    
    //if(command.buttons&BUT_RT)
    //{
      //debugSerial.listen();
     // debugSerial.println("RT");
      //debugSerial.listen();
    //}
    
    if(command.buttons != commanderData.buttons) {
      commanderData.buttons = command.buttons;
      commanderData.dataChanged = true;
    }

    if(command.walkV != commanderData.walkV) {
      commanderData.walkV = command.walkV;
      commanderData.dataChanged = true;
    }

    if(command.walkH != commanderData.walkH) {
      commanderData.walkH = command.walkH;
      commanderData.dataChanged = true;
    }

    if(command.walkV != commanderData.walkV) {
      commanderData.walkV = command.walkV;
      commanderData.dataChanged = true;
    }

    if(command.lookH != commanderData.lookH) {
      commanderData.lookH = command.lookH;
      commanderData.dataChanged = true;
    }

    if(command.lookV != commanderData.lookV) {
      commanderData.lookV = command.lookV;
      commanderData.dataChanged = true;
    }

    //If the commander data has changed then fill out the
    //custom sysex byte array and send it.    
    if(commanderData.dataChanged == true) {
      customSysExLength = 6;
      aryCustomSysEx[0] = commanderData.walkV;
      aryCustomSysEx[1] = commanderData.walkH;
      aryCustomSysEx[2] = commanderData.lookV;
      aryCustomSysEx[3] = commanderData.lookH;
      aryCustomSysEx[4] = commanderData.buttons;
      int checksum = (~(commanderData.walkV + commanderData.walkH + 
                        commanderData.lookV + commanderData.lookH + 
                        commanderData.buttons)) & 0xFF;
      aryCustomSysEx[5] = checksum;
                        
#ifdef DEBUG_SERIAL
      debugSerial.print("Commander ");
      debugSerial.print(", WalkV: "); debugSerial.print(commanderData.walkV); 
      debugSerial.print(", WalkH: "); debugSerial.print(commanderData.walkH); 
      debugSerial.print(", LookV: "); debugSerial.print(commanderData.lookV); 
      debugSerial.print(", LookH: "); debugSerial.print(commanderData.lookH); 
      debugSerial.print(", Buttons: "); debugSerial.print(commanderData.buttons); 
      debugSerial.print(", checksum: "); debugSerial.print(checksum); 
      debugSerial.print ("\n");
#endif
      
      // send commander data
      Firmata.sendSysex(SYSEX_COMMANDER_DATA, customSysExLength, aryCustomSysEx);
      
      commanderData.dataChanged = false;
    } 
  }  
}
#endif 

void setup() 
{  
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  Firmata.begin(115200); //57600 256000 230400 115200
  systemResetCallback();  // reset to default config

#ifdef ENABLE_COMMANDER
  command.begin(38400);

  #ifdef DEBUG_SERIAL 
    debugSerial.println("Setup finished");
    debugSerial.print("Firmata Version: ");
    debugSerial.print(FIRMATA_MAJOR_VERSION); debugSerial.print(".");
    debugSerial.print(FIRMATA_MINOR_VERSION); debugSerial.print(".");
    debugSerial.println(FIRMATA_BUGFIX_VERSION);
  #endif
#else 
  #ifdef DEBUG_SERIAL
    debugSerial.begin(38400);
    debugSerial.println("Setup finished");
    debugSerial.print("Firmata Version: ");
    debugSerial.print(FIRMATA_MAJOR_VERSION); debugSerial.print(".");
    debugSerial.print(FIRMATA_MINOR_VERSION); debugSerial.print(".");
    debugSerial.println(FIRMATA_BUGFIX_VERSION);
  #endif
#endif

}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop() 
{
  byte pin, analogPin;

  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  checkDigitalInputs();  
  
  checkDynamixelMotors();
  
#ifdef ENABLE_COMMANDER  
  checkCommander();
#endif
  
  /* SERIALREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while(Firmata.available())
    Firmata.processInput();

  /* SEND FTDI WRITE BUFFER - make sure that the FTDI buffer doesn't go over
   * 60 bytes. use a timer to sending an event character every 4 ms to
   * trigger the buffer to dump. */

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    // ANALOGREAD - do all analogReads() at the configured sampling interval 
    for(pin=0; pin<TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
      }
    }
  }
}
