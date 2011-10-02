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

	bool recvSensorData(SerialFixedResp & r);

};

/***********************************************************************
 * 
 * MOTORCONTROLLERSERIAL MSG CLASSES
 * 
 ***********************************************************************/

class MCCmdVel : public SerialFixedMsg
{
public:
	
	MCCmdVel(int velocity_l, int velocity_r)
		:SerialFixedMsg(5)
	{
		if(velocity_l > 65535 || velocity_l < -65535 || velocity_r > 65535 || velocity_r < -65535 )
		{
			//exception
		}
		
		data[0] = 0xF1;
		data[1] = velocity_l >> 8;
		data[2] = velocity_l & (0xFF);
		data[3]	= velocity_r >> 8;
		data[4] = velocity_r & (0xFF);

	}
	
	virtual string PrettyData() const
	{
		return "MCCmdVel";
	}
};

class MCSensorData : public SerialFixedResp
{
public:
	
	MCSensorData() : SerialFixedResp(6) {}
	
	virtual string PrettyData() const
	{
		return "MCRespSensor";
	}
	
	int Left()
	{		

		int ret = 0;
		
		ret = data[0] << 8;
		ret = ret | ((unsigned int)data[1]);		

		ret = signextend<signed int,16>(ret);

		return ret;
	}
	
	int Right()
	{			
		
		int ret = 0;
		
		ret = data[2] << 8;
		ret = ret | ((unsigned int)data[3]);

		ret = signextend<signed int,16>(ret);

		return ret;
	}
	
	int Sonar()
	{
		int ret = 0;
		
		ret = data[4] << 8;
		ret = ret | ((unsigned int)data[5]);

		ret = signextend<signed int,16>(ret);

		return ret;
	}
};


#endif

