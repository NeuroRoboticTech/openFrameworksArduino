#include "ArduinoTest.h"


ArduinoTest::ArduinoTest(void)
{
	// listen for EInitialized notification. this indicates that
	// the arduino is ready to receive commands and it is safe to
	// call setupArduino()
	m_EInitializedConnection = this->EInitialized.connect(boost::bind(&ArduinoTest::setupArduino, this, _1));
	m_bSetupArduino	= false;	// flag so we setup arduino when its ready, you don't need to touch this :)
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
    this->sendDigitalPinMode(2, ARD_INPUT);

    // set pin A0 to analog input
    this->sendAnalogPinReporting(0, ARD_ANALOG);
    
    // set pin D13 as digital output
	this->sendDigitalPinMode(13, ARD_OUTPUT);
    // set pin A4 as digital output
    this->sendDigitalPinMode(18, ARD_OUTPUT);  // pin 20 if using StandardFirmata from Arduino 0022 or older

    // set pin D11 as PWM (analog output)
	this->sendDigitalPinMode(11, ARD_PWM);
    
    // attach a servo to pin D9
    // servo motors can only be attached to pin D3, D5, D6, D9, D10, or D11
    this->sendServoAttach(9);
	this->sendServo(9, 0, true);

    // Listen for changes on the digital and analog pins
	m_EDigitalPinChanged = this->EDigitalPinChanged.connect(boost::bind(&ArduinoTest::digitalPinChanged, this, _1));
	m_EAnalogPinChanged = this->EAnalogPinChanged.connect(boost::bind(&ArduinoTest::analogPinChanged, this, _1));
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
    std::cout << "digital pin: " << pinNum << " = " << iVal << "\r\n";

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
