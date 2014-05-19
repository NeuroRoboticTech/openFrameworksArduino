#pragma once

#ifdef WIN32
    #define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
    // Windows Header Files:
    #include <windows.h>

    #define ARDUINO_PORT __declspec( dllimport )
#else
    #define ARDUINO_PORT
#endif

#include "ofArduino.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

class ArduinoTest : public ofArduino
{
protected:
	boost::signals2::connection m_EInitializedConnection;
	boost::signals2::connection m_EDigitalPinChanged;
	boost::signals2::connection m_EAnalogPinChanged;

	void digitalPinChanged(const int & pinNum);
	void analogPinChanged(const int & pinNum);

public:
	ArduinoTest(void);
	virtual ~ArduinoTest(void);

	bool m_bSetupArduino;

	void setupArduino(const int & version);
};

