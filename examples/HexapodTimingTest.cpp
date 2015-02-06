#include "HexapodTimingTest.h"

#ifdef INCLUDE_TIMING

#define TC_ID 1
#define CF_ID 2
#define GRIP_ID 3

HexapodTimingTest::HexapodTimingTest(void)
{
	srand (time(NULL));

	// listen for EInitialized notification. this indicates that
	// the arduino is ready to receive commands and it is safe to
	// call setupArduino()
	m_EInitializedConnection = this->EInitialized.connect(boost::bind(&HexapodTimingTest::setupArduino, this, _1));
	m_bSetupArduino	= false;	// flag so we setup arduino when its ready, you don't need to touch this :)
	m_iTick = 0;
	m_iTicksTillMotorSend = 1000;

	m_iCFPos = CF_CENTER;
	m_iFTPos = FT_CENTER;
	m_iTTPos = TT_CENTER;

	m_iCFVel = 25;
	m_iFTVel = 25;
	m_iTTVel = 25;

	m_lStartTick= 0;
	m_lEndTick= 0;
	m_lTotalCount= -1;
	m_dblTotalMillis = 0;

	m_lStartUpdateTick = 0;
	m_lEndUpdateTick = 0;
	m_dblUpdateTime = 1;  //milliseconds

	m_iTransmitErrors = 0;

	HexapodLeg frontRightLeg(1, 3, 5);
	HexapodLeg frontLeftLeg(2, 4, 6);

	HexapodLeg middleRightLeg(13, 15, 17);
	HexapodLeg middleLeftLeg(14, 16, 18);

	HexapodLeg backRightLeg(7, 9, 11);
	HexapodLeg backLeftLeg(8, 10, 12);

	m_aryLegs.push_back(backLeftLeg);
	m_aryLegs.push_back(backRightLeg);

	m_aryLegs.push_back(middleLeftLeg);
	m_aryLegs.push_back(middleRightLeg);

	m_aryLegs.push_back(frontLeftLeg);
	m_aryLegs.push_back(frontRightLeg);
}


HexapodTimingTest::~HexapodTimingTest(void)
{
}

void HexapodTimingTest::setupArduino(const int & version)
{
    // it is now safe to send commands to the Arduino
    m_bSetupArduino = true;
    
    // print firmware name and version to the console
    std::cout << this->getFirmwareName(); 
    std::cout << "firmata v" << this->getMajorFirmwareVersion() << "." << this->getMinorFirmwareVersion();
        
    // Listen for changes on the digital and analog pins
	m_EDynamixelKeyReceived = this->EDynamixelKeyReceived.connect(boost::bind(&HexapodTimingTest::dynamixelRecieved, this, _1));
	m_EDynamixelAllReceived = this->EDynamixelAllReceived.connect(boost::bind(&HexapodTimingTest::dynamixelRecieved, this, _1));
	m_EDynamixelTransmitError = this->EDynamixelTransmitError.connect(boost::bind(&HexapodTimingTest::dynamixelTransmitError, this, _1, _2));
	m_EDynamixelGetRegister = this->EDynamixelGetRegister.connect(boost::bind(&HexapodTimingTest::dynamixelGetRegister, this, _1, _2, _3));

	//First lets attach to all the leg motors so we can recieve updates.
	int iLegCount=m_aryLegs.size();
	for(int iLegIdx=0; iLegIdx<iLegCount; iLegIdx++)
		m_aryLegs[iLegIdx].AttachServos(this);

	//Now Reset all legs to the default starting position
	for(int iLegIdx=0; iLegIdx<iLegCount; iLegIdx++)
		m_aryLegs[iLegIdx].CenterMotorsAdd(this, 100);
	this->sendDynamixelSynchMoveExecute();
	
	//Wait for three seconds.
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
}

void HexapodTimingTest::shutdownArduino()
{
	int iLegCount=m_aryLegs.size();
	for(int iLegIdx=0; iLegIdx<iLegCount; iLegIdx++)
		m_aryLegs[iLegIdx].DettachServos(this);

	//Now Reset all legs to the default starting position
	for(int iLegIdx=0; iLegIdx<iLegCount; iLegIdx++)
		m_aryLegs[iLegIdx].CenterMotorsAdd(this, 100);
	this->sendDynamixelSynchMoveExecute();

	std::cout << "Transmit errors : " << m_iTransmitErrors << "\r\n";
}

void HexapodTimingTest::update()
{
	ofArduino::update();

	if(m_bSetupArduino)
	{
		m_lEndUpdateTick = osg::Timer::instance()->tick();
		double dblDiff = osg::Timer::instance()->delta_m(m_lStartUpdateTick, m_lEndUpdateTick);
		if(dblDiff > m_dblUpdateTime)
		{
			CalculateNextMove();

			int iLegCount=m_aryLegs.size();
			for(int iLegIdx=0; iLegIdx<iLegCount; iLegIdx++)
				m_aryLegs[iLegIdx].SynchMoveAdd(this, m_iCFPos, m_iCFVel, m_iFTPos, m_iFTVel, m_iTTPos, m_iTTVel);
			this->sendDynamixelSynchMoveExecute();

			m_lStartUpdateTick = osg::Timer::instance()->tick();
		}
	}
	else
		m_lStartUpdateTick = osg::Timer::instance()->tick();

}

//--------------------------------------------------------------
void HexapodTimingTest::dynamixelRecieved(const int & servo) 
{
	if(servo == 1)
	{
		if(m_lTotalCount >= 0)
		{
			m_lEndTick = osg::Timer::instance()->tick();

			double dblMillis = osg::Timer::instance()->delta_m(m_lStartTick, m_lEndTick);

			m_dblTotalMillis+=dblMillis;
			m_lTotalCount++;

			if(m_lTotalCount >= 10)
			{
				double dblAvg = m_dblTotalMillis/m_lTotalCount;
				//std::cout << "Millis: " << dblAvg << ", Total: " << m_lTotalCount << "\r\n";
				//std::cout << "Avg: " << dblAvg << "\r\n";
				std::cout << dblAvg << "\r\n";
				m_lTotalCount = 0;
				m_dblTotalMillis= 0;
			}
		}
	}

	_dynamixelServos[servo]._keyChanged = false;
	_dynamixelServos[servo]._allChanged = false;

	if(servo == 1)
	{
		m_lStartTick = osg::Timer::instance()->tick();
		if(m_lTotalCount<0)
			m_lTotalCount = 0;
	}
}


int HexapodTimingTest::CalculateNextPosIncrement(int iPos, int iMaxChange, int iCenterPos)
{
	int iIncrement = rand() % MAX_STEP;
	int iSign = rand() % 100;
		
	if(iSign < 50)
		iIncrement*=-1; 

	return iIncrement;
}

int HexapodTimingTest::CalculateNextVelIncrement(int iVel, int iMaxVel)
{
	int iIncrement = rand() % MAX_STEP;
	int iSign = rand() % 100;
		
	if(iSign < 50)
		iIncrement*=-1; 

	return iIncrement;
}

void HexapodTimingTest::CheckValues(int &iPos, int iMaxPosOffset, int iCenterPos, int &iVel, int iMaxVel)
{
	if(iPos < (iCenterPos - iMaxPosOffset))
		iPos = (iCenterPos - iMaxPosOffset);
	if(iPos > (iCenterPos + iMaxPosOffset))
		iPos = (iCenterPos + iMaxPosOffset);
		
	if(iVel < 0)
		iVel = 0;
	if(iVel > iMaxVel)
		iVel = iMaxVel;
}

void HexapodTimingTest::CalculateNextMove()
{
	int iPosIncr = CalculateNextPosIncrement(m_iCFPos, CF_MAX_POS_OFFSET, CF_CENTER);
	int iVelIncr = CalculateNextVelIncrement(m_iCFPos, CF_MAX_VEL);
	m_iCFPos+=iPosIncr;
	m_iCFVel+=iVelIncr;
	CheckValues(m_iCFPos, CF_MAX_POS_OFFSET, CF_CENTER, m_iCFVel, CF_MAX_VEL);

	iPosIncr = CalculateNextPosIncrement(m_iFTPos, FT_MAX_POS_OFFSET, FT_CENTER);
	iVelIncr = CalculateNextVelIncrement(m_iFTPos, FT_MAX_VEL);
	m_iFTPos+=iPosIncr;
	m_iFTVel+=iVelIncr;
	CheckValues(m_iFTPos, FT_MAX_POS_OFFSET, FT_CENTER, m_iFTVel, FT_MAX_VEL);

	iPosIncr = CalculateNextPosIncrement(m_iTTPos, TT_MAX_POS_OFFSET, TT_CENTER);
	iVelIncr = CalculateNextVelIncrement(m_iTTPos, TT_MAX_VEL);
	m_iTTPos+=iPosIncr;
	m_iTTVel+=iVelIncr;
	CheckValues(m_iTTPos, TT_MAX_POS_OFFSET, TT_CENTER, m_iTTVel, TT_MAX_VEL);
}

void HexapodTimingTest::dynamixelTransmitError(const int & cmd, const int & servoNum) {

	m_iTransmitErrors++;
	//std::cout << "Transmit error Cmd: " << cmd << ", servo: " << servoNum << "\r\n";
}

void HexapodTimingTest::dynamixelGetRegister(const unsigned char &servo, const unsigned char &reg, const unsigned int &value) {

	std::cout << "Get Register Servo: " << servo << ", reg: " << reg << ", value: " << value << "\r\n";
}

#endif
