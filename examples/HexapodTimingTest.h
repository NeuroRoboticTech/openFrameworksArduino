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

#define CF_CENTER 512
#define FT_CENTER 512
#define TT_CENTER 512

#define CF_MAX_POS_OFFSET 50
#define FT_MAX_POS_OFFSET 50
#define TT_MAX_POS_OFFSET 50

#define CF_MAX_VEL 50
#define FT_MAX_VEL 50
#define TT_MAX_VEL 50

#define MAX_STEP 5

class HexapodLeg
{
public:
	int m_iCFId;
	int m_iFTId;
	int m_iTTId;

	HexapodLeg()
	{
		m_iCFId=0;
		m_iFTId=1;
		m_iTTId=2;
	}
	HexapodLeg(int iCFId, int iFTId, int iTTId)
	{
		m_iCFId=iCFId;
		m_iFTId=iFTId;
		m_iTTId=iTTId;
	}

	virtual ~HexapodLeg(void) {};

	void AttachServos(ofArduino *ard)
	{
		ard->sendDynamixelServoAttach(m_iCFId);
		ard->sendDynamixelServoAttach(m_iFTId);
		ard->sendDynamixelServoAttach(m_iTTId);
	}

	void DettachServos(ofArduino *ard)
	{
		ard->sendDynamixelServoDetach(m_iCFId);
		ard->sendDynamixelServoDetach(m_iFTId);
		ard->sendDynamixelServoDetach(m_iTTId);
	}

	void CenterMotorsAdd(ofArduino *ard, int iVel)
	{
		ard->sendDynamixelSynchMoveAdd(m_iCFId, CF_CENTER, iVel);
		ard->sendDynamixelSynchMoveAdd(m_iFTId, FT_CENTER, iVel);
		ard->sendDynamixelSynchMoveAdd(m_iTTId, TT_CENTER, iVel);
	}

	void SynchMoveAdd(ofArduino *ard, int iPos, int iVel)
	{
		ard->sendDynamixelSynchMoveAdd(m_iCFId, iPos, iVel);
		ard->sendDynamixelSynchMoveAdd(m_iFTId, iPos, iVel);
		ard->sendDynamixelSynchMoveAdd(m_iTTId, iPos, iVel);
	}

	void SynchMoveAdd(ofArduino *ard, int iCFPos, int iCFVel, int iFTPos, int iFTVel, int iTTPos, int iTTVel)
	{
		ard->sendDynamixelSynchMoveAdd(m_iCFId, iCFPos, iCFVel);
		ard->sendDynamixelSynchMoveAdd(m_iFTId, iFTPos, iFTVel);
		ard->sendDynamixelSynchMoveAdd(m_iTTId, iTTPos, iTTVel);
	}
};

class HexapodTimingTest : public ofArduino
{
protected:
	boost::signals2::connection m_EInitializedConnection;
	boost::signals2::connection m_EDynamixelKeyReceived;
	boost::signals2::connection m_EDynamixelAllReceived;
	boost::signals2::connection m_EDynamixelTransmitError;
	boost::signals2::connection m_EDynamixelGetRegister;

	std::vector<HexapodLeg> m_aryLegs;

	int m_iTick;
	int m_iTicksTillMotorSend;

	int m_iCFPos;
	int m_iFTPos;
	int m_iTTPos;

	int m_iCFVel;
	int m_iFTVel;
	int m_iTTVel;

	int m_iTransmitErrors;

	void dynamixelRecieved(const int & servoNum);
	void dynamixelTransmitError(const int & cmd, const int & servoNum);
	void dynamixelGetRegister(const unsigned char &servo, const unsigned char &reg, const unsigned int &value);


	int CalculateNextPosIncrement(int iPos, int iMaxChange, int iCenterPos);
	int CalculateNextVelIncrement(int iVel, int iMaxVel);
	void CheckValues(int &iPos, int iMaxPosOffset, int iCenterPos, int &iVel, int iMaxVel);
	void CalculateNextMove();

	unsigned long long m_lStartUpdateTick;
	unsigned long long m_lEndUpdateTick;
	double m_dblUpdateTime;

	unsigned long long m_lStartTick;
	unsigned long long m_lEndTick;
	double m_dblTotalMillis;
	long m_lTotalCount;

public:
	HexapodTimingTest(void);
	virtual ~HexapodTimingTest(void);

	bool m_bSetupArduino;

	void setupArduino(const int & version);
	void shutdownArduino();

	virtual void update();
};

#endif
