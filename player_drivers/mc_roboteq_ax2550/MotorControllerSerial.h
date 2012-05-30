/*
 *  MotorControllerSerial.h
 *  MotorControllerSerial
 *
 */

#ifndef MOTORCONTROLLERSERIAL_H
#define MOTORCONTROLLERSERIAL_H

//parent class
#include "../driverlib/SerialFixedMsg.h"
#include "../driverlib/SerialFixedResp.h"
#include "../driverlib/SerialDevice.h"

//c++ libs
#include <string>

//c libs
#include <math.h>


using namespace std;


template <typename T, unsigned B>
inline T signextend(const T x)
{
  struct {T x:B;} s;
  return s.x = x;
}


class MotorControllerSerial : public SerialDevice
{
public:
	static const double AXEL_LENGTH_M = 1.0; // Length of Axel

	// send/recv functions
	bool recvUntilMsg(const vector<unsigned char> & msg, unsigned int maxBytes);
	bool recvUntilWatchdog(unsigned int maxBytes);
	bool enterSerialMode();
	bool restart();
	bool recvSensorData();
	bool recvBatteryVoltage();
	bool recvMotorValues();
	bool recvInputs();
	bool sendMotorCmd(double new_left, double new_right);

	// sensor read functions
	double left();
	double right();
	double mainVoltage();
	double internalVoltage();
	bool estopState();

protected:

	double _left;
	double _right;
	double _mainVoltage;
	double _internalVoltage;
	int _inputs[3];

	unsigned char evenParity(unsigned char d);
	int hexStringToInt(const string & hex);
	string intToHexString(int i);

	bool sendBytes(const vector<unsigned char> & data);
	bool recvBytes(vector<unsigned char> & data);
	bool sendSingleMotorCmd(double val, int motorInd);

};

#endif

