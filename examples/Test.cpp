#include "ArduinoTest.h"

int main(int argc, char** argv) 
{
	ArduinoTest ard;
	bool bForward = false;
	int iMotorCount = 0;

	if(!ard.connect("COM4", 57600))
	{
		std::cout << "Failed to connect to arduino!";
		return -1;
	}

	//Need to do this to init the pins, get the firmware version, and  call setupArduino.
	//Will stay in update loop looking for signal. When it arrives Setup will be called
	//and we can start processing.
	ard.sendFirmwareVersionRequest();

	for(int i=0; i<1000000; i++, iMotorCount++)
	{
		std::cout << "i: " << i << "\r\n";

		ard.update();
		boost::this_thread::sleep(boost::posix_time::milliseconds(5));

		if(ard.m_bSetupArduino && iMotorCount > 100)
		{
			iMotorCount = 0;
			bForward = !bForward;

			if(bForward)
			{
				//rotate servo head to 180 degrees
				std::cout << "motor: 180\r\n";
				//ard.sendServo(9, 180);
			}
			else
			{
				//rotate servo head to 0 degrees
				std::cout << "motor: 0\r\n";
				//ard.sendServo(9, 0);
			}
		}


	}

	return 0;
}