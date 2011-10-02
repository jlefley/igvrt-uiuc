/*
 *  Roomba.h
 *  roomba
 *
 */

#ifndef ROOMBA_H
#define ROOMBA_H

//parent class
#include "../driverlib/SerialFixedMsg.h"
#include "../driverlib/SerialFixedResp.h"
#include "../driverlib/SerialDevice.h"

//c++ libs
#include <string>


using namespace std;

class Roomba : public SerialDevice
{
public:
	enum eMode { OFF, PASSIVE, SAFE, FULL };

	static const int DIAMETER_MM = 330; // total diameter of roomba (including shell)
	static const int AXLE_LENGTH_MM = 258; // axel length of roomba (wheel center to wheel center)
	static const int TVEL_MAX_MM_S = 500; // maximum translational velocity of center of axel
	static const int RADIUS_MAX_MM = 2000; // maximum turning radius

	bool wakeUp();
	bool changeMode(eMode mode);
	bool changeBaud(unsigned int baudCode);
};



/***********************************************************************
 * 
 * ROOMBA MSG CLASSES
 * 
 ***********************************************************************/

class CmdBaud : public SerialFixedMsg
{
public:
	
	CmdBaud(unsigned int baudCode)
		:SerialFixedMsg(2)
	{
		data[0] = 129;
		data[1] = 10;
		
		if(baudCode >= 0 && baudCode <= 11)
			data[1] = baudCode;
	}
	
	virtual string PrettyData() const
	{
		return "Baud";
	}
};


class CmdClean : public SerialFixedMsg
{
public:
	CmdClean()
	{
		data[0] = 135;
	}
	
	virtual string PrettyData() const
	{
		return "Clean";
	}
};


class CmdControl : public SerialFixedMsg
{
public:
	CmdControl()
	{
		data[0] = 130;
	}
	
	virtual string PrettyData() const
	{
		return "Control";
	}
};


class CmdDrive : public SerialFixedMsg
{
public:

	CmdDrive(int velocity, int radius)
		:SerialFixedMsg(5)
	{
		data[0] = 137;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
		
		if(velocity > Roomba::TVEL_MAX_MM_S)
			velocity = Roomba::TVEL_MAX_MM_S;
		else if(velocity < ((-1)*Roomba::TVEL_MAX_MM_S))
			velocity = ((-1)*Roomba::TVEL_MAX_MM_S);
		
		if(radius != 32768 && radius != 1 && radius != -1)
		{
			if(radius > Roomba::RADIUS_MAX_MM)
				radius = Roomba::RADIUS_MAX_MM;
			else if(radius < ((-1)*Roomba::RADIUS_MAX_MM))
				radius = ((-1)*Roomba::RADIUS_MAX_MM);
		}
		
		data[1] = velocity >> 8;
		data[2] = velocity & 0x00FF;
		
		data[3] = radius >> 8;
		data[4] = radius & 0x00FF;
	}
	
	virtual string PrettyData() const
	{
		return "Drive";
	}
	
private:
};


class CmdDriveStraight : public CmdDrive
{
public:

	CmdDriveStraight(int velocity) : CmdDrive(velocity, 32768) {}
	
	virtual string PrettyData() const
	{
		return "Drive Straight";
	}
};


class CmdFull : public SerialFixedMsg
{
public:
	CmdFull()
	{
		data[0] = 132;
	}
	
	virtual string PrettyData() const
	{
		return "Full";
	}
};


class CmdLED : public SerialFixedMsg
{
public:
	enum eStatus {STATUS_OFF, STATUS_RED, STATUS_GREEN, STATUS_AMBER};
	
	
	CmdLED(bool dirtDetect, bool max, bool clean, bool spot, eStatus status, unsigned int power_color, unsigned int power_intensity)
		:SerialFixedMsg(4)
	{
		data[0] = 139;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		
		if(dirtDetect)
			data[1] |= 1;
		
		if(max)
			data[1] |= 2;
		
		if(clean)
			data[1] |= 4;
		
		if(spot)
			data[1] |= 8;
		
		switch(status)
		{
		case STATUS_OFF:
			data[1] |= (0 << 4);
			break;
		case STATUS_RED:
			data[1] |= (1 << 4);
			break;
		case STATUS_GREEN:
			data[1] |= (2 << 4);
			break;
		case STATUS_AMBER:
			data[1] |= (3 << 4);
			break;
		}
		
		if(power_color > 255)
			data[2] = 255;
		else
			data[2] = power_color;
			
		if(power_intensity > 255)
			data[3] = 255;
		else
			data[3] = power_intensity;
		
	}
	
	virtual string PrettyData() const
	{
		return "LED";
	}
};


class CmdMotors : public SerialFixedMsg
{
public:
	enum eStatus {STATUS_OFF, STATUS_RED, STATUS_GREEN, STATUS_AMBER};
	
	
	CmdMotors(bool mainbrush, bool vacuum, bool sidebrush)
		:SerialFixedMsg(2)
	{
		data[0] = 138;
		data[1] = 0;
		if (sidebrush) data[1]++;
		if (vacuum)    data[1]=data[1]+2;
		if (mainbrush) data[1] = data[1]+4;
		
		
	}
	
	virtual string PrettyData() const
	{
		return "MOTOR";
	}
};


class CmdPlay : public SerialFixedMsg
{
public:

	CmdPlay(int songIndex)
		:SerialFixedMsg(2)
	{
		data[0] = 141;
		data[1] = songIndex;
				
	}
	
	virtual string PrettyData() const
	{
		return "Play";
	}
};


class CmdPower : public SerialFixedMsg
{
public:
	CmdPower()
	{
		data[0] = 133;
	}
	
	virtual string PrettyData() const
	{
		return "Power";
	}
};


class CmdSafe : public SerialFixedMsg
{
public:
	CmdSafe()
	{
		data[0] = 131;
	}
	
	virtual string PrettyData() const
	{
		return "Safe";
	}
};


class CmdSensors : public SerialFixedMsg
{
public:

	CmdSensors(unsigned int packetCode)
		:SerialFixedMsg(2)
	{
		data[0] = 142;
		data[1] = 0;
		
		if(packetCode >= 0 && packetCode <= 3)
			data[1] = packetCode;
	}
	
	virtual string PrettyData() const
	{
		return "Sensors";
	}
};


class CmdSong : public SerialFixedMsg
{
public:
	CmdSong(const vector<unsigned int> & notes, const vector<unsigned int> & noteLength, int songIndex)
		:SerialFixedMsg(2*notes.size() + 3)
	{
		if(notes.size() != noteLength.size())
			throw runtime_error("CmdSong: length of notes and noteLength vectors do not match");

		data[0] = 140;
		data[1] = songIndex;
		data[2] = notes.size();
		
		for(unsigned int i = 0; i < notes.size(); i++)
		{
			data[2*i+3] = notes[i];
			data[2*i+4] = noteLength[i];
		}
		
				
	}
	
	virtual string PrettyData() const
	{
		return "LED";
	}
};


class CmdStart : public SerialFixedMsg
{
public:
	CmdStart()
	{
		data[0] = 128;
	}
	
	virtual string PrettyData() const
	{
		return "Start";
	}
};


class CmdStop : public CmdDrive
{
public:

	CmdStop() : CmdDrive(0, 0) {}
	
	virtual string PrettyData() const
	{
		return "Stop";
	}
};


class CmdTurnInPlace : public CmdDrive
{
public:

	CmdTurnInPlace(int velocity) : CmdDrive(velocity, 1) {}
	
	virtual string PrettyData() const
	{
		return "Turn In Place";
	}
};


class RespSensor0 : public SerialFixedResp
{
public:
	
	enum eChargingState {NOT_CHARGING, CHARGING_RECOVERY, CHARGING, TRICKLE_CHARGING, WAITING, CHARGING_ERROR};
	
	RespSensor0() : SerialFixedResp(26) {}
	
	virtual string PrettyData() const
	{
		return "Sensor 0 Response";
	}
	
	bool bumpRight()
	{
		return ((data[0] & 0x01) > 0);
	}
	
	bool bumpLeft()
	{
		return ((data[0] & 0x02) > 0);
	}
	
	bool wheeldropRight()
	{
		return ((data[0] & 0x04) > 0);
	}
	
	bool wheeldropLeft()
	{
		return ((data[0] & 0x08) > 0);
	}
	
	bool wheeldropCaster()
	{
		return ((data[0] & 0x10) > 0);
	}
	
	bool wall()
	{
		return (data[1] == 1);
	}
	
	bool cliffLeft()
	{
		return (data[2] == 1);
	}
	
	bool cliffFrontLeft()
	{
		return (data[3] == 1);
	}
	
	bool cliffFrontRight()
	{
		return (data[4] == 1);
	}
	
	bool cliffRight()
	{
		return (data[5] == 1);
	}
	
	bool virtualWall()
	{
		return (data[6] == 1);
	}
	
	bool motorOvercurrentsSideBrush()
	{
		return ((data[7] & 0x01) > 0);
	}
	
	bool motorOvercurrentsVacuum()
	{
		return ((data[7] & 0x02) > 0);
	}
	
	bool motorOvercurrentsMainBrush()
	{
		return ((data[7] & 0x04) > 0);
	}
	
	bool motorOvercurrentsDriveRight()
	{
		return ((data[7] & 0x08) > 0);
	}
	
	bool motorOvercurrentsDriveLeft()
	{
		return ((data[7] & 0x10) > 0);
	}
	
	unsigned int dirtDetectorLeft()
	{
		return data[8];
	}
	
	unsigned int dirtDetectorRight()
	{
		return data[9];
	}
	
	unsigned int remoteControlCommand()
	{
		return data[10];
	}

	bool buttonMax()
	{
		return ((data[11] & 0x01) > 0);
	}
	
	bool buttonClean()
	{
		return ((data[11] & 0x02) > 0);
	}
	
	bool buttonSpot()
	{
		return ((data[11] & 0x04) > 0);
	}
	
	bool buttonPower()
	{
		return ((data[11] & 0x08) > 0);
	}
	
	//D = (Sr + Sl)/2 [mm]
	int distance()
	{
		int16_t val = (data[12] << 8) | data[13];
		
		return val;
	}
	
	//A = (Sr - Sl)/2 [mm]
	int angle()
	{
		int16_t val = (data[14] << 8) | data[15];
		
		return val;
	}
	
	// [rad]
	double angleRad()
	{
		return (2 * ((double)angle())) / Roomba::AXLE_LENGTH_MM;
	}

	// distance right wheel has traveled (Sr) [mm]
	int rightDistance()
	{
		return (distance() + angle());
	}

	// distance right wheel has traveled (Sl) [mm]
	int leftDistance()
	{
		return (distance() - angle());
	}
	
	eChargingState chargingState()
	{
		switch(data[16])
		{
		case 0:
			return NOT_CHARGING;
			break;
		case 1:
			return CHARGING_RECOVERY;
			break;
		case 2:
			return CHARGING;
			break;
		case 3:
			return TRICKLE_CHARGING;
			break;
		case 4:
			return WAITING;
			break;
		case 5:
			return CHARGING_ERROR;
			break;
		}
		
		return NOT_CHARGING;
	}
	
	unsigned int voltage()
	{
		return (data[17] << 8) | data[18];
	}
	
	int current()
	{
		int16_t val = (data[19] << 8) | data[20];
		
		return val;
	}

	int temperature()
	{
		int8_t val = data[21];
		
		return val;
	}
	
	unsigned int charge()
	{
		return (data[22] << 8) | data[23];
	}
	
	unsigned int capacity()
	{
		return (data[24] << 8) | data[25];
	}
	
};


#endif

