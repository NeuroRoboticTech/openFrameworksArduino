#include "stdafx.h"
#include "PlaybackMovements.h"
#include <fstream>
#include <iostream>
#include <vector>

PlaybackMovements::PlaybackMovements(void)
{
	m_EInitializedConnection = this->EInitialized.connect(boost::bind(&PlaybackMovements::setupArduino, this, _1));
	m_bSetupArduino	= false;	// flag so we setup arduino when its ready, you don't need to touch this :)
	m_bDone = false;
	m_iCurrentTimeStep = 0;
	numServos = 4;

#ifdef INCLUDE_TIMING
	m_lStartTick= 0;
	m_lEndTick= 0;
	m_lTotalCount= -1;
	m_dblTotalMillis = 0;
#endif
}


PlaybackMovements::~PlaybackMovements(void)
{
}

void PlaybackMovements::setupArduino(const int & version)
{
	std::cout << "setupArduino() \n";
	
	m_EInitializedConnection.disconnect();
        
	// print firmware name and version to the console
	std::cout << this->getFirmwareName(); 
	std::cout << "firmata v" << this->getMajorFirmwareVersion() << "." << this->getMinorFirmwareVersion() << "\n";

	loadKinematicData("C:\\Projects\\AnimatLabSDK\\3rdParty\\openFrameworksArduino\\playback\\desired_pos.txt");
	loadTime("C:\\Projects\\AnimatLabSDK\\3rdParty\\openFrameworksArduino\\playback\\time.txt");

	servoIDs.push_back(1);
	servoIDs.push_back(2);
	servoIDs.push_back(3);
	servoIDs.push_back(4);

	servoLLs.push_back(0);
	servoLLs.push_back(0);
	servoLLs.push_back(0);
	servoLLs.push_back(0);

	servoULs.push_back(1023);
	servoULs.push_back(1023);
	servoULs.push_back(1023);
	servoULs.push_back(1023);

	// Listen for changes on the digital and analog pins
	m_EDynamixelKeyReceived = this->EDynamixelKeyReceived.connect(boost::bind(&PlaybackMovements::dynamixelRecieved, this, _1));
	m_EDynamixelAllReceived = this->EDynamixelAllReceived.connect(boost::bind(&PlaybackMovements::dynamixelRecieved, this, _1));
	m_EDynamixelTransmitError = this->EDynamixelTransmitError.connect(boost::bind(&PlaybackMovements::dynamixelTransmitError, this, _1, _2));
	m_EDynamixelGetRegister = this->EDynamixelGetRegister.connect(boost::bind(&PlaybackMovements::dynamixelGetRegister, this, _1, _2, _3));

	// Listen for changes on the digital and analog pins
	for (int i=0;i<numServos;i++){
		std::cout << "Attaching servo " << servoIDs[i] << ".\n";
		this->sendDynamixelServoAttach(servoIDs[i]);
		this->sendDynamixelConfigureServo(servoIDs[i],servoLLs[i],servoULs[i],1023,1,1,1,32,32);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

	m_iCurrentTimeStep = 0;

	//Set the intial servo positions
	for (int j=0;j<numServos;j++)
		sendDynamixelSynchMoveAdd(servoIDs[j], convertFromMx64ToAx12(servoCommands[m_iCurrentTimeStep][j]), 50);
	sendDynamixelSynchMoveExecute();

	m_iCurrentTimeStep++;

	outfile.open("C:\\Projects\\AnimatLabSDK\\3rdParty\\openFrameworksArduino\\playback\\recorded_positions.txt");

	// it is now safe to send commands to the Arduino
	m_bSetupArduino = true;
}

std::vector<int> PlaybackMovements::parseLine(std::string line) 
{
    std::istringstream iss(line);
	std::vector<std::string> tokens;
	std::copy(std::istream_iterator<std::string>(iss),
		 std::istream_iterator<std::string>(),
		 back_inserter(tokens));

	std::vector<int> iVect(tokens.size());
	std::transform(tokens.begin(), tokens.end(), iVect.begin(), 
				[](const std::string &arg) { return std::stof(arg); });
	return iVect;
}

void PlaybackMovements::loadKinematicData(std::string filename)
{
    std::string line;
    std::ifstream infile;
    infile.open (filename, std::ifstream::in);

	if(!infile.is_open())
		throw std::runtime_error(std::string("Unable to open file: ") + filename);

	servoCommands.clear();
    std::string previousLine="";
    while(!infile.eof()) // To get you all the lines.
    {
        getline(infile,line); 
		std::vector<int> values = parseLine(line); 

		servoCommands.push_back(values);
    }
    infile.close();
}

void PlaybackMovements::loadTime(std::string filename)
{
    std::string line;
    std::ifstream infile;
    infile.open (filename, std::ifstream::in);

	if(!infile.is_open())
		throw std::runtime_error(std::string("Unable to open file: ") + filename);

	timeVecDiff.clear();
    std::string previousLine="";
	double prevTime = -1;
    while(!infile.eof()) // To get you all the lines.
    {
        getline(infile,line); 
		double time = (double) atof(line.c_str());

		if(prevTime >= 0)
		{
			double diff = time - prevTime;
			timeVecDiff.push_back(diff);
		}
		else
			timeVecDiff.push_back(time);

		prevTime = time;
	}
    infile.close();
}

int PlaybackMovements::convertFromMx64ToAx12(int pos)
{
	return (int) (((float) pos / 4096.0) * 1024);
}

void PlaybackMovements::update()
{
	if(m_bSetupArduino)
	{
		if(!m_bDone && m_iCurrentTimeStep <  timeVecDiff.size())
		{
			for (int j=0;j<numServos;j++)
				sendDynamixelSynchMoveAdd(servoIDs[j], convertFromMx64ToAx12(servoCommands[m_iCurrentTimeStep][j]), 50);
			sendDynamixelSynchMoveExecute();

			boost::this_thread::sleep(boost::posix_time::milliseconds(1000*timeVecDiff[m_iCurrentTimeStep]));

			ofArduino::update();

			saveData();

			m_iCurrentTimeStep++;

			if(m_iCurrentTimeStep >=  timeVecDiff.size())
			{
				m_bDone = true;
				outfile.close();
			}
		}
		else
			m_bDone = true;
	}
	else
		ofArduino::update();

}

void PlaybackMovements::saveData()
{
	unsigned int position = 0;
	outfile << m_iCurrentTimeStep;
	for (int j=0;j<numServos;j++){
		position = _dynamixelServos[servoIDs[j]]._actualPosition;
		std::cout << "Motor " << servoIDs[j] << " moved to " << position << "\n";
		if (_dynamixelServos[servoIDs[j]]._keyChanged){
			std::cout << "Motor " << servoIDs[j] << " key data changed. \n";
			//ard._dynamixelServos[servoIDs[j]]._keyChanged = false;
		}
		outfile << "\t" << position;
	}
	outfile << "\n";
}

//dynamixel event handler, called whenever a dynamixel update occurs

//--------------------------------------------------------------
void PlaybackMovements::dynamixelRecieved(const int & servo) 
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

	std::cout << "Data received for " << servo << ". \n";

#ifdef INCLUDE_TIMING
	m_lStartTick = osg::Timer::instance()->tick();
	if(m_lTotalCount<0)
		m_lTotalCount = 0;
#endif
}

void PlaybackMovements::dynamixelTransmitError(const int & cmd, const int & servoNum) {

	std::cout << "Transmit error Cmd: " << cmd << ", servo: " << servoNum << "\r\n";
}

void PlaybackMovements::dynamixelGetRegister(const unsigned char &servo, const unsigned char &reg, const unsigned int &value) {

	std::cout << "Get Register Servo: " << servo << ", reg: " << reg << ", value: " << value << "\r\n";
}
