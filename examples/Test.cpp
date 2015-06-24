#include "stdafx.h"
#include "ArduinoTest.h"
#include "PlaybackMovements.h"

#ifdef TARGET_WIN32
#include "HexapodTimingTest.h"
#endif

int RunArduinoTest()
{
	ArduinoTest ard;
	bool bForward = false;
	int iMotorCount = 0;

#ifdef TARGET_WIN32
	if(!ard.connect("COM6", 256000)) //38400 57600 115200 230400 256000
#else
	if(!ard.connect("ttyUSB0", 57600)) //38400 57600 115200 230400 256000
#endif
	{
		std::cout << "Failed to connect to arduino!";
		return -1;
	}

	//Need to do this to init the pins, get the firmware version, and  call setupArduino.
	//Will stay in update loop looking for signal. When it arrives Setup will be called
	//and we can start processing.
	//ard.sendReset();
	//ard.sendProtocolVersionRequest();
	ard.sendFirmwareVersionRequest();

    //ard.setupArduino(1);

	while(1) //for(int i=0; i<1000000; i++, iMotorCount++)
	{
		//std::cout << "i: " << i << "\r\n";

		ard.update();
		//boost::this_thread::sleep(boost::posix_time::milliseconds(5));

		//if(ard.m_bSetupArduino && iMotorCount > 100)
		//{
		//	iMotorCount = 0;
		//	bForward = !bForward;

		//	if(bForward)
		//	{
		//		//rotate servo head to 180 degrees
		//		std::cout << "motor: 180\r\n";
		//		//ard.sendServo(9, 180);
		//	}
		//	else
		//	{
		//		//rotate servo head to 0 degrees
		//		std::cout << "motor: 0\r\n";
		//		//ard.sendServo(9, 0);
		//	}
		//}


	}
}


int RunPlaybackMovements()
{
	PlaybackMovements ard;

#ifdef TARGET_WIN32
	if(!ard.connect("COM9", 256000)) //38400 57600 115200 230400 256000
#else
	if(!ard.connect("ttyUSB0", 57600)) //38400 57600 115200 230400 256000
#endif
	{
		std::cout << "Failed to connect to arduino!";
		return -1;
	}

	//Need to do this to init the pins, get the firmware version, and  call setupArduino.
	//Will stay in update loop looking for signal. When it arrives Setup will be called
	//and we can start processing.
	ard.sendFirmwareVersionRequest();	

	while(!ard.done()) //for(int i=0; i<1000000; i++, iMotorCount++)
		ard.update();

	return 0;
}

#ifdef WIN32
#ifdef INCLUDE_TIMING

int RunHexapodTimingTest()
{
	HexapodTimingTest ard;
	unsigned long long lStartTick;
	bool bStartedTimer = false;
	double dblUpdateTime = 0;

	if(!ard.connect("COM6", 256000)) //38400 57600 115200 230400 256000
	{
		std::cout << "Failed to connect to arduino!";
		return -1;
	}

	ard.sendFirmwareVersionRequest();

	while(!ard.m_bSetupArduino || (ard.m_bSetupArduino && dblUpdateTime < 5)) //for(int i=0; i<1000000; i++, iMotorCount++)
	{
		//std::cout << "i: " << i << "\r\n";

		ard.update();

		if(ard.m_bSetupArduino && !bStartedTimer)
		{
			bStartedTimer = true;
			lStartTick = osg::Timer::instance()->tick();
		}
		else if(bStartedTimer)
			dblUpdateTime = osg::Timer::instance()->delta_s(lStartTick, osg::Timer::instance()->tick());
	}

	ard.shutdownArduino();

	return 0;
}
#endif
#endif

int main(int argc, char** argv)
{
	//return RunArduinoTest();
	//return RunHexapodTimingTest();
	return RunPlaybackMovements();
}

