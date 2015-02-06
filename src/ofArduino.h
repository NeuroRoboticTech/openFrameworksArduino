/*
 * Copyright 2007-2008 (c) Erik Sjodin, eriksjodin.net
 * Adapted from Wiring version 2011 (c) Carlos Mario Rodriguez and
 * Hernando Barragan by Dominic Amato
 * Adapted from Arbotix Firmata version 2014 (c) David Cofer
 * NeuroRoboticTechnologies, LLC
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <boost/timer.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/bind.hpp>
#include "ofSerialWin.h"
#include "ofSerialLinux.h"

/*
 * Version numbers for the protocol. The protocol is still changing, so these
 * version numbers are important. This number can be queried so that host
 * software can test whether it will be compatible with the currently installed firmware.
 */

#define FIRMATA_MAJOR_VERSION                           2 // for non-compatible changes
#define FIRMATA_MINOR_VERSION                           0 // for backwards compatible changes
#define FIRMATA_MAX_DATA_BYTES                          32 // max number of data bytes in non-Sysex messages
// message command bytes (128-255/0x80-0xFF)
#define FIRMATA_DIGITAL_MESSAGE                         0x90 // send data for a digital pin
#define FIRMATA_ANALOG_MESSAGE                          0xE0 // send data for an analog pin (or PWM)
#define FIRMATA_REPORT_ANALOG                           0xC0 // enable analog input by pin #
#define FIRMATA_REPORT_DIGITAL                          0xD0 // enable digital input by port pair
//
#define FIRMATA_SET_PIN_MODE                            0xF4 // set a pin to INPUT/OUTPUT/PWM/etc
//
#define FIRMATA_REPORT_VERSION                          0xF9 // report protocol version
#define FIRMATA_SYSTEM_RESET                            0xFF // reset from MIDI
//
#define FIRMATA_START_SYSEX                             0xF0 // start a MIDI Sysex message
#define FIRMATA_END_SYSEX                               0xF7 // end a MIDI Sysex message
// pin modes
#define FIRMATA_INPUT                                   0x00
#define FIRMATA_OUTPUT                                  0x01
#define FIRMATA_ANALOG                                  0x02 // analog pin in analogInput mode
#define FIRMATA_PWM                                     0x03 // digital pin in PWM output mode
#define FIRMATA_SERVO                                   0x04 // digital pin in Servo output mode
#define SHIFT											0x05 // shiftIn/shiftOut mode
#define I2C												0x06 // pin included in I2C setup
#define FIRMATA_INPUT_PULLUP							0x07 // pull-up resistors enabled
#define TOTAL_PIN_MODES 8
// extended command set using SysEx (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for custom commands */
#define FIRMATA_SYSEX_SERVO_CONFIG                      0x70 // set max angle, minPulse, maxPulse, freq
#define FIRMATA_SYSEX_FIRMATA_STRING					0x71 // a string message with 14-bits per char
#define SHIFT_DATA										0x75 // a bitstram to/from a shift register
#define I2C_REQUEST										0x76 // send an I2C read/write request
#define I2C_REPLY										0x77 // a reply to an I2C request
#define I2C_CONFIG										0x78 // config I2C settings such as delay times and power pins
#define EXTENDED_ANALOG									0x6F // analog write (PWM, Servo, etc) to any pin
#define PIN_STATE_QUERY									0x6D // ask for a pin's current mode and value
#define PIN_STATE_RESPONSE								0x6E // reply with pin's current mode and value
#define CAPABILITY_QUERY								0x6B // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE								0x6C // reply with supported modes and resolution
#define ANALOG_MAPPING_QUERY							0x69 // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE							0x6A // reply with mapping info
#define FIRMATA_SYSEX_REPORT_FIRMWARE					0x79 // report name and version of the firmware
#define SAMPLING_INTERVAL								0x7A // set the poll rate of the main loop
#define FIRMATA_SYSEX_NON_REALTIME                      0x7E // MIDI Reserved for non-realtime messages
#define FIRMATA_SYSEX_REALTIME                          0x7F // MIDI Reserved for realtime messages

#define MAX_DYNAMIXEL_SERVOS							50
#define DYNAMIXEL_KEY_DATA_LENGTH						6*2+1
#define DYNAMIXEL_ALL_DATA_LENGTH						10*2+1
#define DYNAMIXEL_GET_REGISTER_LENGTH					5*2+1
#define COMMANDER_DATA_LENGTH							6*2+1

#define SYSEX_DYNAMIXEL_KEY_SERVO_DATA                      0x68 // Data packet of key (pos, speed) Dynamixel data.
#define SYSEX_DYNAMIXEL_ALL_SERVO_DATA                      0x67 // Data packet of all (pos, speed, load, voltage, temp) Dynamixel data.
#define SYSEX_DYNAMIXEL_CONFIG		                        0x66 // Data packet to configure firmata to listen for dynamixel data.
#define SYSEX_DYNAMIXEL_SYNCH_MOVE_START					0x65 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_SYNCH_MOVE_ADD						0x64 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_SYNCH_MOVE_EXECUTE					0x63 // Data packet to configure up to 5 motors to move using synch move command.
#define SYSEX_DYNAMIXEL_MOVE								0x62 // Data packet to send immediate move command.
#define SYSEX_DYNAMIXEL_STOP								0x61 // Data packet to send immediate move command.
#define SYSEX_DYNAMIXEL_TRANSMIT_ERROR						0x60 // Data packet for when there is a transmission error detected on the arbotix side.
#define SYSEX_DYNAMIXEL_SET_REGISTER						0x59 // Data packet to set a specific register in a Dynamixel servo.
#define SYSEX_DYNAMIXEL_GET_REGISTER						0x58 // Data packet to get a specific register in a Dynamixel servo.
#define SYSEX_DYNAMIXEL_CONFIGURE_SERVO						0x57 // Data packet to configure key motor params like cw and ccw limits.
#define SYSEX_DYNAMIXEL_STOPPED								0x56 // Data packet to configure reporting of when a servo stops moving.
#define SYSEX_COMMANDER_DATA								0x55 // Data packet with commander remote control buttons pressed.

// ---- arduino constants (for Arduino NG and Diecimila)

// board settings
#define ARD_TOTAL_DIGITAL_PINS							22 // total number of pins currently supported
#define ARD_TOTAL_ANALOG_PINS							6
#define ARD_TOTAL_PORTS                                 3 // total number of ports for the board
// pin modes
#define ARD_INPUT                                       0x00
#define ARD_OUTPUT                                      0x01
#define ARD_ANALOG                                      0x02 // analog pin in analogInput mode
#define ARD_PWM                                         0x03 // digital pin in PWM output mode
#define ARD_SERVO                                       0x04 // digital pin in Servo output mode
#define ARD_INPUT_PULLUP                                0x07 // pull-up rsistors enabled
#define ARD_HIGH                                        1
#define ARD_LOW                                         0
#define ARD_ON                                          1
#define ARD_OFF                                         0

/*
 #if defined(__AVR_ATmega168__)  // Arduino NG and Diecimila
 #define ARD_TOTAL_ANALOG_PINS       8
 #define ARD_TOTAL_DIGITAL_PINS      22 // 14 digital + 8 analog
 #define ARD_TOTAL_PORTS             3 // total number of ports for the board
 #define ARD_ANALOG_PORT             2 // port# of analog used as digital
 #elif defined(__AVR_ATmega8__)  // old Arduinos
 #define ARD_TOTAL_ANALOG_PINS       6
 #define ARD_TOTAL_DIGITAL_PINS      20 // 14 digital + 6 analog
 #define ARD_TOTAL_PORTS             3  // total number of ports for the board
 #define ARD_ANALOG_PORT             2  // port# of analog used as digital
 #elif defined(__AVR_ATmega128__)// Wiring
 #define ARD_TOTAL_ANALOG_PINS       8
 #define ARD_TOTAL_DIGITAL_PINS      43
 #define ARD_TOTAL_PORTS             5 // total number of ports for the board
 #define ARD_ANALOG_PORT             2 // port# of analog used as digital
 #else // anything else
 #define ARD_TOTAL_ANALOG_PINS       6
 #define ARD_TOTAL_DIGITAL_PINS      14
 #define ARD_TOTAL_PORTS             3 // total number of ports for the board
 #define ARD_ANALOG_PORT             2 // port# of analog used as digital
 #endif
 */

// DEPRECATED as of firmata v2.2
#define SYSEX_SERVO_ATTACH                      0x00
#define SYSEX_SERVO_DETACH                      0x01
#define SYSEX_SERVO_WRITE                       0x02

#define OF_ARDUINO_DELAY_LENGTH					10.0

#define FIRMWARE2_2								22
#define FIRMWARE2_3                             23


/**
        This class extend ofStandardFirmata and provides additional functionality like servo support through SysEx messages.
		use the OFstdFirmata for servo support...

**/

class ARDUINO_PORT ofDynamixelData {
public:
	bool _keyChanged;
	bool _allChanged;
	unsigned int _id;
	unsigned int _goalPosition;
	unsigned int _actualPosition;
	unsigned int _goalSpeed;
	unsigned int _actualSpeed;
	unsigned int _load;
	unsigned char _temperature;
	unsigned char _voltage;
	bool _moving;
	unsigned char _LED;
	unsigned char _alarm;

	ofDynamixelData()
	{
		_keyChanged = false;
		_allChanged = false;
		_id = 0;
		_goalPosition = 0;
		_actualPosition = 0;
		_goalSpeed = 0;
		_actualSpeed = 0;
		_load = 0;
		_temperature = 0;
		_voltage = 0;
		_moving = false;
		_LED = 0;
		_alarm = 0;
	};
};

class ARDUINO_PORT ofCommanderData {
public:
	bool _changed;
	signed char _walkV;
	signed char _walkH;
	signed char _lookV;
	signed char _lookH;
	unsigned char _buttons;
	unsigned char _ext;

	ofCommanderData()
	{
		_changed = false;
		_walkV = 0;
		_walkH = 0;
		_lookV = 0;
		_lookH = 0;
		_buttons = 0;
		_ext = 0;
	};
};


class ARDUINO_PORT ofArduino{

        public:
                ofArduino();

                virtual ~ofArduino();


                // --- setup functions
				virtual bool connect(std::string device, int baud = 57600);
				// opens a serial port connection to the arduino

				virtual void disconnect();
				// closes the serial port connection

				virtual bool isArduinoReady();

				virtual void  setUseDelay(bool bDelay);

				virtual void update();
				// polls data from the serial port, this has to be called periodically

				virtual bool isInitialized();
				// returns true if a succesfull connection has been established and the Arduino has reported a firmware

				virtual void setDigitalHistoryLength(int length);
				virtual void setAnalogHistoryLength(int length);
				virtual void setStringHistoryLength(int length);
				virtual void setSysExHistoryLength(int nSysEx);

				// --- senders

				virtual void sendDigitalPinMode(int pin, int mode);
				// pin: 2-13
				// mode: ARD_INPUT, ARD_OUTPUT, ARD_PWM
				// setting a pins mode to ARD_INPUT turns on reporting for the port the pin is on
				// Note: analog pins 0-5 can be used as digitial pins 16-21 but if the mode of _one_ of these pins is set to ARD_INPUT then _all_ analog pin reporting will be turned off

				virtual void sendAnalogPinReporting(int pin, int mode);
				// pin: 0-5
				// mode: ARD_ON or ARD_OFF
				// Note: analog pins 0-5 can be used as digitial pins 16-21 but if reporting for _one_ analog pin is enabled then reporting for _all_ of digital pin 16-21 will be turned off

				virtual void sendDigital(int pin, int value, bool force = false);
				// pin: 2-13
				// value: ARD_LOW or ARD_HIGH
				// the pins mode has to be set to ARD_OUTPUT or ARD_INPUT (in the latter mode pull-up resistors are enabled/disabled)
				// Note: pin 16-21 can also be used if analog inputs 0-5 are used as digital pins

				virtual void sendPwm(int pin, int value, bool force = false);
				// pin: 3, 5, 6, 9, 10 and 11
				// value: 0 (always off) to 255 (always on).
				// the pins mode has to be set to ARD_PWM
				// TODO check if the PWM bug still is there causing frequent digital port reporting...

				virtual void sendSysEx(int command, std::vector<unsigned char> data);

				virtual void sendString(std::string str);
				// firmata can not handle strings longer than 12 characters.

				virtual void sendProtocolVersionRequest();

				virtual void sendFirmwareVersionRequest();

				virtual void sendReset();

				// --- senders for SysEx communication

				virtual void sendSysExBegin();
				// sends the FIRMATA_START_SYSEX command

				virtual void sendSysExEnd();
				// sends the FIRMATA_END_SYSEX command

				virtual void sendByte(unsigned char value);
				// sends a byte without wrapping it in a firmata message, data has to be in the 0-127 range,
				// values > 127 will be interpreted as commands.

				virtual void sendValueAsTwo7bitBytes(int value);
				// sends a value as two 7-bit bytes without wrapping it in a firmata message
				// values in the range 0 - 16384 will be sent as two bytes within the 0-127 data range.

				// --- getters

				virtual int getPwm(int pin);
				// pin: 3, 5, 6, 9, 10 and 11
				// returns the last set PWM value (0-255) for the given pin
				// the pins mode has to be ARD_PWM
				// Note: pin 16-21 can also be used if analog inputs 0-5 are used as digital pins

				virtual int getDigital(int pin);
				// pin: 2-13
				// returns the last received value (if the pin mode is ARD_INPUT) or the last set value (if the pin mode is ARD_OUTPUT) for the given pin
				// Note: pin 16-21 can also be used if analog inputs 0-5 are used as digital pins

				virtual int getAnalog(int pin);
				// pin: 0-5
				// returns the last received analog value (0-1023) for the given pin

				virtual std::vector<unsigned char> getSysEx();
				// returns the last received SysEx message

				virtual std::string getString();
				// returns the last received string

				virtual int getMajorProtocolVersion();
				// returns the major firmware version

				virtual int getMinorProtocolVersion();
				// returns the minor firmware version

				virtual int getMajorFirmwareVersion();
				// returns the major firmware version

				virtual int getMinorFirmwareVersion();
				// returns the minor firmware version

				virtual std::string getFirmwareName();
				// returns the name of the firmware

				virtual std::list<int>* getDigitalHistory(int pin);
				// pin: 2-13
				// returns a pointer to the digital data history list for the given pin
				// Note: pin 16-21 can also be used if analog inputs 0-5 are used as digital pins

				virtual std::list<int>* getAnalogHistory(int pin);
				// pin: 0-5
				// returns a pointer to the analog data history list for the given pin

				virtual std::list<std::vector<unsigned char> >* getSysExHistory();
				// returns a pointer to the SysEx history

				virtual std::list<std::string>* getStringHistory();
				// returns a pointer to the string history

				virtual int makeWord(unsigned char low, unsigned char  high);
				//Combines two bytes into a word

				virtual unsigned char  getLowByte(int val);
				//Gets the low byte of an int

				virtual unsigned char  getHighByte(int val);
				//gets the high byte of an int.

				ofDynamixelData _dynamixelServos[MAX_DYNAMIXEL_SERVOS];

				ofCommanderData _commanderData;

				int getDigitalPinMode(int pin);
				// returns ARD_INPUT, ARD_OUTPUT, ARD_PWM, ARD_SERVO, ARD_ANALOG

				int getAnalogPinReporting(int pin);
				// returns ARD_ON, ARD_OFF

				int getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb);
				// useful for parsing SysEx messages

				unsigned int getByteFromDataIterator(std::vector<unsigned char>::iterator &it, std::vector<unsigned char>::iterator &end);
				// useful for parsing SysEx messages

				unsigned int GetWordFromDataIterator(std::vector<unsigned char>::iterator &it, std::vector<unsigned char>::iterator &end);
				// useful for parsing SysEx messages

				// --- events

				boost::signals2::signal<void (const int)> EDigitalPinChanged;
				// triggered when a digital pin changes value, the pin that changed is passed as an argument

				boost::signals2::signal<void (const int)> EAnalogPinChanged;
				// triggered when an analog pin changes value, the pin that changed is passed as an argument

				boost::signals2::signal<void (const std::vector<unsigned char>)> ESysExReceived;
				// triggered when a SysEx message that isn't in the extended command set is received, the SysEx message is passed as an argument

				boost::signals2::signal<void (const int)> EProtocolVersionReceived;
				// triggered when a protocol version is received, the major version is passed as an argument

				boost::signals2::signal<void (const int)> EFirmwareVersionReceived;
				// triggered when a firmware version is received, the major version is passed as an argument

				boost::signals2::signal<void (const int)> EInitialized;
				// triggered when the firmware version is received upon connect, the major firmware version is passed as an argument
				// from this point it's safe to send to the Arduino.

				boost::signals2::signal<void (const std::string)> EStringReceived;
				// triggered when a string is received, the string is passed as an argument

				boost::signals2::signal<void (const int)> EDynamixelAllReceived;
				// triggered when a dynamixel data update packet is received, the servo ID is passed as an argument

				boost::signals2::signal<void (const int)> EDynamixelKeyReceived;
				// triggered when a dynamixel data update packet is received, the servo ID is passed as an argument

				boost::signals2::signal<void (const int, const int)> EDynamixelTransmitError;
				// triggered when the arbotix gets a transmission error like an invalid checksum

				boost::signals2::signal<void (const unsigned char, const unsigned char, const unsigned int)> EDynamixelGetRegister;
				// triggered when the arbotix sends back a register value from one of the Dynamixel motors.

				boost::signals2::signal<void (const int)> EDynamixelStopped;
				// triggered when a dynamixel stopped packet is received, the servo ID is passed as an argument

				boost::signals2::signal<void (const int)> ECommanderDataReceived;
				// triggered when a commander data update packet is received, the servo ID is passed as an argument

				// -- servo
			    virtual void sendServo(int pin, int value, bool force=false);
                // pin: 9, 10
				// the pin has to have a servo attached

				// angle parameter DEPRECATED as of Firmata 2.2
                virtual void sendServoAttach(int pin, int minPulse=544, int maxPulse=2400, int angle=180);
				// pin: 9, 10
                // attaches a servo to a pin

				// sendServoDetach DEPRECATED as of Firmata 2.2
                virtual void sendServoDetach(int pin);
				// pin: 9, 10
                // detaches a servo from a pin, the pin mode remains as OUTPUT

				virtual int getServo(int pin);
				// returns the last set servo value for a pin if the pin has a servo attached

				//Tells an arbotix board to monitor a given servo and report back its data.
				virtual void sendDynamixelServoAttach(unsigned char servo);

				//Tells an arbotix board to quit monitoring a given servo and report back its data.
				virtual void sendDynamixelServoDetach(unsigned char servo);

				//Sends a SynchMove start command to the Arbotix telling it that a new set of move commands are coming.
				virtual void sendDynamixelSynchMoveStart();

				//Transmits the command to setup a motor as part of a synch motor move command.
				//You can setup any number of moves to participate in this move command and then
				//call sendDynamixelSynchMoveExecute to trigger the movement.
				virtual void sendDynamixelSynchMoveAdd(unsigned char servo, int pos, int speed);

				//Transmits the command to move the servos that have been setup using addDynamixelSynchMove
				virtual void sendDynamixelSynchMoveExecute();

				//Transmits the command to move a single motor. Does not use the synch move.
				virtual void sendDynamixelMove(unsigned char servo, int pos, int speed);

				//Transmits the stop to move a single motor. When the Arbotix recieves this command it will
				//slow the servo down to is slowest setting, then quickly query the servo for its current
				//position and the set the goal position to be the current position to stop it from moving.
				virtual void sendDynamixelStop(unsigned char servo);

				//Transmits the command to set a byte of the servo register.
				virtual void sendDynamixelSetRegister(unsigned char servo, unsigned char reg, unsigned char length, unsigned int value);

				//Transmits the command to get a byte of the servo register.
				virtual void sendDynamixelGetRegister(unsigned char servo, unsigned char reg, unsigned char length);

				//Transmits the command to get a byte of the servo register.
				virtual void sendDynamixelConfigureServo(unsigned char servo, unsigned int cwlimit, unsigned int ccwlimit, 
												 unsigned int maxtorque, unsigned char delaytime, 
												 unsigned char cwcomplmargin, unsigned char ccwcomplmargin,
												 unsigned char cwcomplslope, unsigned char ccwcomplslope);

				//Transmits the command to check if the servo is moving and when it is no longer moving send a signal back.
				virtual void sendDynamixelStopped(unsigned char servo);

				virtual bool waitForSysExMessage(unsigned char cmd, unsigned int timeout_sec = 1);

		protected:
				bool _initialized;

                void initPins();
                int _totalDigitalPins;

				virtual void sendDigitalPinReporting(int pin, int mode);
				// sets pin reporting to ARD_ON or ARD_OFF
				// enables / disables reporting for the pins port

				virtual void sendDigitalPortReporting(int port, int mode);
				// sets port reporting to ARD_ON or ARD_OFF
				// enables / disables reporting for ports 0-2
				// port 0: pins 2-7  (0,1 are serial RX/TX)
				// port 1: pins 8-13 (14,15 are disabled for the crystal)
				// port 2: pins 16-21 analog pins used as digital, all analog reporting will be turned off if this is set to ARD_ON

				virtual void processData(unsigned char inputData);
				virtual void processDigitalPort(int port, unsigned char value);
				virtual void processSysExData(std::vector<unsigned char> data);

				virtual void checkIncomingSysExMessage(unsigned char cmd);

				ofSerial _port;
				int _portStatus;

				// --- history variables
				int _analogHistoryLength;
				int _digitalHistoryLength;
				int _stringHistoryLength;
				int _sysExHistoryLength;

				// --- data processing variables
				int _waitForData;
				int _executeMultiByteCommand;
				int _multiByteChannel; // indicates which pin data came from

				// --- data holders
				unsigned char _storedInputData[FIRMATA_MAX_DATA_BYTES];
				std::vector<unsigned char> _sysExData;
				int _majorProtocolVersion;
				int _minorProtocolVersion;
				int _majorFirmwareVersion;
				int _minorFirmwareVersion;
				std::string _firmwareName;
				bool _firmwareReceived;

				// sum of majorFirmwareVersion * 10 + minorFirmwareVersion
				int _firmwareVersionSum;

				std::list<std::vector<unsigned char> > _sysExHistory;
				// maintains a history of received sysEx messages (excluding SysEx messages in the extended command set)

				std::list<std::string> _stringHistory;
				// maintains a history of received strings

				std::list<int> _analogHistory[ARD_TOTAL_ANALOG_PINS];
				// a history of received data for each analog pin

				std::list<int> _digitalHistory[ARD_TOTAL_DIGITAL_PINS];
				// a history of received data for each digital pin

				int _digitalPinMode[ARD_TOTAL_DIGITAL_PINS];
				// the modes for all digital pins

				int _digitalPinValue[ARD_TOTAL_DIGITAL_PINS];
				// the last set values (DIGITAL/PWM) on all digital pins

				int _digitalPortValue[ARD_TOTAL_PORTS];
				// the last set values on all ports

				int _digitalPortReporting[ARD_TOTAL_PORTS];
				// whether pin reporting is enabled / disabled

				int _digitalPinReporting[ARD_TOTAL_DIGITAL_PINS];
				// whether pin reporting is enabled / disabled

				int _analogPinReporting[ARD_TOTAL_ANALOG_PINS];
				// whether pin reporting is enabled / disabled

				bool bUseDelay;

				bool connected;

				int _servoValue[ARD_TOTAL_DIGITAL_PINS];
                // the last set servo values

				boost::timer _Timer;

				//Keeps track of how many synch move adds have been added since last execute or start
				int _dynamixelMoveAdds;

				//A specific SysEx message we are waiting on. -1 if not waiting.
				int _waitingForSysExMessage;

				//set to true when the message we are waiting on is found. false otherwise
				bool _sysExMessageFound;
};

typedef ofArduino ofStandardFirmata;

