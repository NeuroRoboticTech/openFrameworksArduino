#include "stdafx.h"
#include "ArduinoTest.h"

#define TC_ID 1
#define CF_ID 2
#define GRIP_ID 3

ArduinoTest::ArduinoTest(void)
{
	// listen for EInitialized notification. this indicates that
	// the arduino is ready to receive commands and it is safe to
	// call setupArduino()
	m_EInitializedConnection = this->EInitialized.connect(boost::bind(&ArduinoTest::setupArduino, this, _1));
	m_bSetupArduino	= false;	// flag so we setup arduino when its ready, you don't need to touch this :)
	m_bSwingLeg = false;

#ifdef INCLUDE_TIMING
	m_lStartTick= 0;
	m_lEndTick= 0;
	m_lTotalCount= -1;
	m_dblTotalMillis = 0;
#endif
}


ArduinoTest::~ArduinoTest(void)
{
}

void ArduinoTest::setupArduino(const int & version)
{
	//m_EInitializedConnection.disconnect();
    
    // it is now safe to send commands to the Arduino
    m_bSetupArduino = true;
    
    // print firmware name and version to the console
    std::cout << this->getFirmwareName(); 
    std::cout << "firmata v" << this->getMajorFirmwareVersion() << "." << this->getMinorFirmwareVersion();
        
    // Note: pins A0 - A5 can be used as digital input and output.
    // Refer to them as pins 14 - 19 if using StandardFirmata from Arduino 1.0.
    // If using Arduino 0022 or older, then use 16 - 21.
    // Firmata pin numbering changed in version 2.3 (which is included in Arduino 1.0)
    
    // set pins D2 and A5 to digital input
    //this->sendDigitalPinMode(2, ARD_INPUT);

    // set pin A0 to analog input
    //this->sendAnalogPinReporting(1, ARD_ANALOG);
    
    // set pin D13 as digital output
	//this->sendDigitalPinMode(13, ARD_OUTPUT);
    // set pin A4 as digital output
   // this->sendDigitalPinMode(18, ARD_OUTPUT);  // pin 20 if using StandardFirmata from Arduino 0022 or older

    // set pin D11 as PWM (analog output)
	//this->sendDigitalPinMode(11, ARD_PWM);
    
    // attach a servo to pin D9
    // servo motors can only be attached to pin D3, D5, D6, D9, D10, or D11
    //this->sendServoAttach(9);
	//this->sendServo(9, 0, true);

	//Send a sysex to report a dynamixel servo
	//std::vector<unsigned char> sysexData;
	//sysexData.push_back(3);
	//sysexData.push_back(1);
	//this->sendSysEx(SYSEX_DYNAMIXEL_CONFIG, sysexData);

    // Listen for changes on the digital and analog pins
	m_EDigitalPinChanged = this->EDigitalPinChanged.connect(boost::bind(&ArduinoTest::digitalPinChanged, this, _1));
	m_EAnalogPinChanged = this->EAnalogPinChanged.connect(boost::bind(&ArduinoTest::analogPinChanged, this, _1));
	m_ECommanderChanged = this->ECommanderDataReceived.connect(boost::bind(&ArduinoTest::commanderChanged, this, _1));
	m_EDynamixelKeyReceived = this->EDynamixelKeyReceived.connect(boost::bind(&ArduinoTest::dynamixelRecieved, this, _1));
	m_EDynamixelAllReceived = this->EDynamixelAllReceived.connect(boost::bind(&ArduinoTest::dynamixelRecieved, this, _1));
	m_EDynamixelTransmitError = this->EDynamixelTransmitError.connect(boost::bind(&ArduinoTest::dynamixelTransmitError, this, _1, _2));
	m_EDynamixelGetRegister = this->EDynamixelGetRegister.connect(boost::bind(&ArduinoTest::dynamixelGetRegister, this, _1, _2, _3));

	//Set the CW and CCW limits to be in a valid range for this test
	this->sendDynamixelSetRegister(TC_ID, 0x06, 2, 160);
	this->sendDynamixelSetRegister(TC_ID, 0x08, 2, 810);

	this->sendDynamixelGetRegister(TC_ID, 0x06, 2);
	bool ret = this->waitForSysExMessage(SYSEX_DYNAMIXEL_GET_REGISTER, 2);
	this->sendDynamixelGetRegister(TC_ID, 0x08, 2);
	ret = this->waitForSysExMessage(SYSEX_DYNAMIXEL_GET_REGISTER, 2);
	ret = this->waitForSysExMessage(SYSEX_DYNAMIXEL_SET_REGISTER, 2);

	this->sendDynamixelConfigureServo(TC_ID, 170, 800, 1023, 1, 1, 1, 32, 32); 

	//Test waiting for motor to stop
	this->sendDynamixelMove(TC_ID, 512, 0);
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));

	this->sendDynamixelMove(TC_ID, 200, 40);
	this->sendDynamixelStopped(TC_ID);
	ret = this->waitForSysExMessage(SYSEX_DYNAMIXEL_STOPPED, 10);

	this->sendDynamixelServoAttach(TC_ID);
	this->sendDynamixelServoAttach(CF_ID);
	this->sendDynamixelServoAttach(GRIP_ID);

	this->sendDynamixelMove(GRIP_ID, 400, 0);
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));

	this->sendDynamixelMove(CF_ID, 400, 0);
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));

	this->sendDynamixelMove(TC_ID, 400, 0);
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));

	//Start a move then stop it.
	this->sendDynamixelMove(TC_ID, 700, 20);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	this->sendDynamixelStop(TC_ID);

	//Wait for a second.
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	m_bSwingLeg = false;
	this->sendDynamixelSynchMoveAdd(TC_ID, 512, 100);
	this->sendDynamixelSynchMoveAdd(CF_ID, 512, 100);
	this->sendDynamixelSynchMoveAdd(GRIP_ID, 512, 100);
	this->sendDynamixelSynchMoveExecute();

	//Wait for a second.
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	//Get the motor moving slowly over to the side
	this->sendDynamixelSynchMoveAdd(TC_ID, 180, 100);
	this->sendDynamixelSynchMoveAdd(CF_ID, 650, 100);
	this->sendDynamixelSynchMoveAdd(GRIP_ID, 650, 100);
	this->sendDynamixelSynchMoveExecute();
	m_bSwingLeg = true;
}


// digital pin event handler, called whenever a digital pin value has changed
// note: if an analog pin has been set as a digital pin, it will be handled
// by the digitalPinChanged function rather than the analogPinChanged function.

//--------------------------------------------------------------
void ArduinoTest::digitalPinChanged(const int & pinNum) 
{
    // do something with the digital input. here we're simply going to print the pin number and
    // value to the screen each time it changes
	int iVal = this->getDigital(pinNum);
    //std::cout << "digital pin: " << pinNum << " = " << iVal << "\r\n";

	this->sendDigital(13, iVal);
}

// analog pin event handler, called whenever an analog pin value has changed

//--------------------------------------------------------------
void ArduinoTest::analogPinChanged(const int & pinNum) 
{
    // do something with the analog input. here we're simply going to print the pin number and
    // value to the screen each time it changes
	int iVal = this->getAnalog(pinNum);
    std::cout << "analog pin: " << pinNum << " = " << iVal << "\r\n";
}

//commander event handler, called whenever a commander button value has changed

//--------------------------------------------------------------
void ArduinoTest::commanderChanged(const int & pinNum) 
{
 //   // do something with the analog input. here we're simply going to print the pin number and
 //   // value to the screen each time it changes
	//int iVal = this->getAnalog(pinNum);
    std::cout << "Commander changed: buttons" << this->_commanderData._buttons << 
		" walkV " << (int) this->_commanderData._walkV <<
		" walkH " << (int) this->_commanderData._walkH <<
		" lookV " << (int) this->_commanderData._lookV <<
		" lookH " << (int) this->_commanderData._lookH <<
		" buttons " << (int) this->_commanderData._buttons << "\r\n";
}

//dynamixel event handler, called whenever a dynamixel update occurs

//--------------------------------------------------------------
void ArduinoTest::dynamixelRecieved(const int & servo) 
{
#ifdef INCLUDE_TIMING
	if(m_lTotalCount >= 0)
	{
		m_lEndTick = osg::Timer::instance()->tick();

		double dblMillis = osg::Timer::instance()->delta_m(m_lStartTick, m_lEndTick);

		m_dblTotalMillis+=dblMillis;
		m_lTotalCount++;

		if(m_lTotalCount >= 100)
		{
			double dblAvg = m_dblTotalMillis/m_lTotalCount;
			//std::cout << "Millis: " << dblAvg << ", Total: " << m_lTotalCount << "\r\n";
		    std::cout << "Avg: " << dblAvg << "\r\n";
			m_lTotalCount = 0;
			m_dblTotalMillis= 0;
		}
	}
#endif

 //   // do something with the analog input. here we're simply going to print the pin number and
 //   // value to the screen each time it changes
	//if(servo == TC_ID)
	//    std::cout << "servo: " << servo << ", Pos: " << _dynamixelServos[servo]._actualPosition << ", swing: " << m_bSwingLeg  << "\r\n";

	_dynamixelServos[servo]._keyChanged = false;
	_dynamixelServos[servo]._allChanged = false;

	if(	m_bSwingLeg && servo == TC_ID && _dynamixelServos[servo]._actualPosition <= (185))
	{
		m_bSwingLeg = false;
		this->sendDynamixelSynchMoveAdd(TC_ID, 800, 100);
		this->sendDynamixelSynchMoveAdd(CF_ID, 512, 100);
		this->sendDynamixelSynchMoveAdd(GRIP_ID, 512, 100);
		this->sendDynamixelSynchMoveExecute();
	    std::cout << "Swing\r\n";
	}
	else if(!m_bSwingLeg && servo == TC_ID && _dynamixelServos[servo]._actualPosition >= (795))
	{
		m_bSwingLeg = true;
		this->sendDynamixelSynchMoveAdd(TC_ID, 180, 100);
		this->sendDynamixelSynchMoveAdd(CF_ID, 650, 100);
		this->sendDynamixelSynchMoveAdd(GRIP_ID, 650, 100);
		this->sendDynamixelSynchMoveExecute();
	    std::cout << "Stance\r\n";
	}

	//this->sendDynamixelServoDetach(servo);

#ifdef INCLUDE_TIMING
	m_lStartTick = osg::Timer::instance()->tick();
	if(m_lTotalCount<0)
		m_lTotalCount = 0;
#endif
}

void ArduinoTest::dynamixelTransmitError(const int & cmd, const int & servoNum) {

	std::cout << "Transmit error Cmd: " << cmd << ", servo: " << servoNum << "\r\n";
}

void ArduinoTest::dynamixelGetRegister(const unsigned char &servo, const unsigned char &reg, const unsigned int &value) {

	std::cout << "Get Register Servo: " << servo << ", reg: " << reg << ", value: " << value << "\r\n";
}
