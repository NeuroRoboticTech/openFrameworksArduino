#pragma once

class ArduinoTest : public ofArduino
{
protected:
	boost::signals2::connection m_EInitializedConnection;
	boost::signals2::connection m_EDigitalPinChanged;
	boost::signals2::connection m_EAnalogPinChanged;
	boost::signals2::connection m_ECommanderChanged;
	boost::signals2::connection m_EDynamixelKeyReceived;
	boost::signals2::connection m_EDynamixelAllReceived;
	boost::signals2::connection m_EDynamixelTransmitError;
	boost::signals2::connection m_EDynamixelGetRegister;

	bool m_bSwingLeg;

	void digitalPinChanged(const int & pinNum);
	void analogPinChanged(const int & pinNum);
	void commanderChanged(const int & pinNum);
	void dynamixelRecieved(const int & servoNum);
	void dynamixelTransmitError(const int & cmd, const int & servoNum);
	void dynamixelGetRegister(const unsigned char &servo, const unsigned char &reg, const unsigned int &value);

#ifdef INCLUDE_TIMING
	unsigned long long m_lStartTick;
	unsigned long long m_lEndTick;
	double m_dblTotalMillis;
	long m_lTotalCount;
#endif

public:
	ArduinoTest(void);
	virtual ~ArduinoTest(void);

	bool m_bSetupArduino;

	void setupArduino(const int & version);
};

