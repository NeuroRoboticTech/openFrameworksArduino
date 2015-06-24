#pragma once

#include "ofArduino.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

class PlaybackMovements : public ofArduino
{
protected:
	boost::signals2::connection m_EInitializedConnection;
	boost::signals2::connection m_EDynamixelKeyReceived;
	boost::signals2::connection m_EDynamixelAllReceived;
	boost::signals2::connection m_EDynamixelTransmitError;
	boost::signals2::connection m_EDynamixelGetRegister;

	bool m_bDone;
	unsigned int m_iCurrentTimeStep;

	//Initialize the servo IDs, their upper and lower limits, and the number of servos.
	//These must be listed in the same order as in the file to play back.
	std::vector<unsigned int> servoIDs;
	std::vector<unsigned int> servoLLs;
	std::vector<unsigned int> servoULs;
	unsigned int numServos;
	std::vector<double> timeVecDiff;

	std::ofstream outfile;

	std::vector< std::vector<int> > servoCommands;

	void loadKinematicData(std::string filename);
	void loadTime(std::string filename);
	int convertFromMx64ToAx12(int pos);

	void dynamixelRecieved(const int & servoNum);
	void dynamixelTransmitError(const int & cmd, const int & servoNum);
	void dynamixelGetRegister(const unsigned char &servo, const unsigned char &reg, const unsigned int &value);

	std::vector<int> parseLine(std::string line);
	void saveData();

#ifdef INCLUDE_TIMING
	unsigned long long m_lStartTick;
	unsigned long long m_lEndTick;
	double m_dblTotalMillis;
	long m_lTotalCount;
#endif

public:
	PlaybackMovements(void);
	virtual ~PlaybackMovements(void);

	bool m_bSetupArduino;

	bool done() {return m_bDone;};
	void setupArduino(const int & version);
	virtual void update();
};