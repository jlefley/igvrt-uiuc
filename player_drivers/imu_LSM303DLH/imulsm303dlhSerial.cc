/*
 *  imulsm303dlhSerial.cc
 *  imulsm303dlh303Serial
 *
 */

#include "imulsm303dlhSerial.h"

bool imulsm303dlhSerial::readMsg()
{
	vector<unsigned char> msg(1);
	string body = "";

	while(recvBytes(msg))
	{
		if(msg[0] == '$')
			break;
	}

	while(recvBytes(msg))
	{
		if(msg[0] != '\n')
		{
			body += msg[0];
		}
		else
		break;
	}

	sscanf(body.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &aX,&aY,&aZ,&mX,&mY,&mZ,&heading,&distancecenter,&distanceleft,&distanceright);
	
	return true;
}

bool imulsm303dlhSerial::flashLight(int flash)
{
	vector<unsigned char> msg(1);

	if(flash == 1)
	{
		msg[0] = 'a';
	}
	else
	{
		msg[0] = 's';
	}

	if(!sendBytes(msg))
	{
		return false;
	}

	return true;
}

int imulsm303dlhSerial::getHeading()
{
	return heading;
}

int imulsm303dlhSerial::getRightDistance()
{
	return distanceright;
}


int imulsm303dlhSerial::getLeftDistance()
{
	return distanceleft;
}

int imulsm303dlhSerial::getCenterDistance()
{
	return distancecenter;
}

/*bool imulsm303dlhSerial::recvSensorData()
{
	if(!recvBatteryVoltage())
		return false;

	if(!recvMotorValues())
		return false;

	return true;
}

bool imulsm303dlhSerial::recvBatteryVoltage()
{
	vector<unsigned char> msg(3);
	vector<unsigned char> resp(6);
	string mainBatteryText = "00";
	string internalText = "00";

	msg[0] = '?';
	msg[1] = 'e';
	msg[2] = '\r';

	if(!sendBytes(msg))
		return false;

	if(!recvUntilMsg(msg, 4096))
		return false;

	if(!recvBytes(resp))
		return false;

	mainBatteryText[0] = resp[0];
	mainBatteryText[1] = resp[1];

	internalText[0] = resp[3];
	internalText[1] = resp[4];

	_mainVoltage = (((double)hexStringToInt(mainBatteryText)) / 256.0)*55.0;
	_internalVoltage = (((double)hexStringToInt(internalText)) / 256.0)*28.5;

	return true;
}

bool imulsm303dlhSerial::recvMotorValues()
{
	vector<unsigned char> msg(3);
	vector<unsigned char> resp(6);
	string motor1Text = "00";
	string motor2Text = "00";

	msg[0] = '?';
	msg[1] = 'v';
	msg[2] = '\r';

	if(!sendBytes(msg))
		return false;

	if(!recvUntilMsg(msg, 4096))
		return false;

	if(!recvBytes(resp))
		return false;

	motor1Text[0] = resp[0];
	motor1Text[1] = resp[1];

	motor2Text[0] = resp[3];
	motor2Text[1] = resp[4];

	_left = (((double)hexStringToInt(motor1Text)) / 127.0)*100.0;
	_right = (((double)hexStringToInt(motor2Text)) / 127.0)*100.0;

	return true;
}

bool imulsm303dlhSerial::sendMotorCmd(double new_left, double new_right)
{
	if(!sendSingleMotorCmd(new_left, 1))
		return false;

	if(!sendSingleMotorCmd(new_right, 2))
		return false;

	return true;
}

bool imulsm303dlhSerial::sendSingleMotorCmd(double val, int motorInd)
{
	vector<unsigned char> msg(5);
	vector<unsigned char> resp(1);
	string val_str = "00";
	int val_scaled;

	//expects motor values to be in range [-100,100]

	if(val > 100)
		val = 100;

	if(val < -100)
		val = -100;

	val_scaled = rint((fabs(val) / 100.0)*127.0);

	val_str = intToHexString(val_scaled);

	msg[0] = '!';
	if(val < 0)
	{
		if(motorInd == 1)
			msg[1] = 'a';
		else
			msg[1] = 'b';
	}
	else
	{
		if(motorInd == 1)
			msg[1] = 'A';
		else
			msg[1] = 'B';
	}
	msg[2] = val_str[0];
	msg[3] = val_str[1];
	msg[4] = '\r';

	if(!sendBytes(msg))
		return false;

	if(!recvUntilMsg(msg, 4096))
		return false;

	if(!recvBytes(resp))
		return false;

	if(resp[0] != '+')
		return false;

	return true;
}

bool imulsm303dlhSerial::recvUntilMsg(const vector<unsigned char> & msg, unsigned int maxBytes)
{
	vector<unsigned char> resp(1);
	bool foundMsgStart = false;

	if(msg.size() == 0)
		return true;

	for(unsigned int i=0; i < maxBytes; i++)
	{
		if(!recvBytes(resp))
			return false;

		if(resp[0] == msg[0])
		{
			foundMsgStart = true;
			break;
		}
	}

	if(!foundMsgStart)
		return false;

	if(msg.size() == 1)
		return true;

	for(unsigned int i=1; i < msg.size(); i++)
	{
		if(!recvBytes(resp))
			return false;

		if(resp[0] != msg[i])
			return false;
	}

	return true;
}

bool imulsm303dlhSerial::recvUntilWatchdog(unsigned int maxBytes)
{
	vector<unsigned char> msg(1);

	msg[0] = 'W';

	return recvUntilMsg(msg, maxBytes);
}

bool imulsm303dlhSerial::enterSerialMode()
{
	vector<unsigned char> msg(10);

	msg[0] = '\r';
	msg[1] = '\r';
	msg[2] = '\r';
	msg[3] = '\r';
	msg[4] = '\r';
	msg[5] = '\r';
	msg[6] = '\r';
	msg[7] = '\r';
	msg[8] = '\r';
	msg[9] = '\r';

	if(!sendBytes(msg))
		return false;

	return recvUntilWatchdog(4096);
}

unsigned char imulsm303dlhSerial::evenParity(unsigned char d)
{
	unsigned char e = 0;

	for(unsigned int i=0; i < 8; i++)
	{
		e = e ^ (d & 0x1);

		d = d >> 1;
	}

	return e;
}

int imulsm303dlhSerial::hexStringToInt(const string & hex)
{
	int i = 0;

	sscanf(hex.c_str(),"%x",&i);

	return i;
}

string imulsm303dlhSerial::intToHexString(int i)
{
	char c_str[3];

	sprintf(c_str, "%02X", i);

	string str(c_str);

	return str;
}

bool imulsm303dlhSerial::sendBytes(const vector<unsigned char> & data)
{
	vector<unsigned char> data_mod(data.size());

	for(unsigned int i=0; i < data.size(); i++)
	{
		data_mod[i] = data[i] | (evenParity(data[i]) << 7);
	}

	return SerialDevice::sendBytes(data_mod);
}

bool imulsm303dlhSerial::recvBytes(vector<unsigned char> & data)
{
	if(SerialDevice::recvBytes(data))
	{
		for(unsigned int i=0; i < data.size(); i++)
		{
			if(evenParity(data[i]) != 0)
				return false;

			data[i] = data[i] & 0x7F;
		}

		return true;
	}

	return false;
}

double imulsm303dlhSerial::left()
{
	return _left;
}

double imulsm303dlhSerial::right()
{
	return _right;
}

double imulsm303dlhSerial::mainVoltage()
{
	return _mainVoltage;
}

double imulsm303dlhSerial::internalVoltage()
{
	return _internalVoltage;
}
*/
