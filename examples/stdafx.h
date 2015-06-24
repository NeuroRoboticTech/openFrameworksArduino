#ifdef WIN32
    #define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
    // Windows Header Files:
    #include <windows.h>

    #define ARDUINO_PORT __declspec( dllimport )
#else
    #define ARDUINO_PORT
#endif

//#define INCLUDE_TIMING

#ifdef INCLUDE_TIMING
	//Include the timer code and openthreads code from osg
	#include <osg/Timer>
	#include <OpenThreads/Thread>
	#ifdef _DEBUG
		#pragma comment(lib, "osgd.lib")
		#pragma comment(lib, "OpenThreadsd.lib")
	#else
		#pragma comment(lib, "osg.lib")
		#pragma comment(lib, "OpenThreads.lib")
	#endif
#endif

#include "ofArduino.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
