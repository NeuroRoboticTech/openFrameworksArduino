#pragma once

#ifdef WIN32
    #define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
    // Windows Header Files:
    #include <windows.h>

    #define ARDUINO_PORT __declspec( dllexport )
#else
    #define ARDUINO_PORT
#endif

#pragma warning(disable:4996) 
