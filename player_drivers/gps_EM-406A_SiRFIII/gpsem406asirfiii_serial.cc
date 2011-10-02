/*
 *  gpsem406asirfiii_serial.cc
 *  gpsem406asirfiii_serial
 *
 */

#include "gpsem406asirfiii_serial.h"

GPSEM406ASIRFIIISerial::GPSEM406ASIRFIIISerial()
{
	_validFix = false;
	_utc_sec = 0;
	_utc_usec = 0;
	_lat = 0;
	_lng = 0;
}

bool GPSEM406ASIRFIIISerial::recvData()
{
	string msg = "";
	vector<string> fields;

	if(!recvMsg(msg))
		return false;

	if(!parseMsg(msg, fields))
		return false;

	if(!saveData(fields))
		return false;

	return true;
}

bool GPSEM406ASIRFIIISerial::recvMsg(string & msg)
{
	vector<unsigned char> resp(1);
	bool foundMsgStart = false;
	bool foundMsgEnd1 = false;
	bool foundMsgEnd2 = false;
	msg = "";

	// wait for '$' (start of message)
	for(unsigned int i=0; i < 4096; i++)
	{
		if(!recvBytes(resp))
			return false;

		if(resp[0] == '$')
		{
			foundMsgStart = true;
			break;
		}
	}

	if(!foundMsgStart)
		return false;
	
	// wait for carriage return (end of message 1) and collect message
	for(unsigned int i=0; i < 4096; i++)
	{
		if(!recvBytes(resp))
			return false;

		if(resp[0] == '\r')
		{
			foundMsgEnd1 = true;
			break;
		}
		
		msg += resp[0];
	}

	if(!foundMsgEnd1)
		return false;

	// wait for line feed
	if(!recvBytes(resp))
		return false;
	
	if(resp[0] == '\n')
	{
		foundMsgEnd2 = true;
	}
	
	// check for end of message
	if(!foundMsgEnd1 || !foundMsgEnd2)
		return false;
	
	return true;
}

bool GPSEM406ASIRFIIISerial::parseMsg(const string & msg, vector<string> & fields)
{
	string field;
	unsigned int i = 0;

	fields.clear();

	while(i < msg.size())
	{
		field = "";

		while(i < msg.size() && msg[i] != ',')
		{
			field += msg[i];
			i++;
		}

		fields.push_back(field);

		i++;
	}

	return true;
}

bool GPSEM406ASIRFIIISerial::saveData(const vector<string> & fields)
{
	if(fields[0] == "GPGGA")
	{
		return saveData_GPGGA(fields);
	}
	
	return true;
}

bool GPSEM406ASIRFIIISerial::saveData_GPGGA(const vector<string> & fields)
{
	_validFix = false;
	if(fields[6] == "")
		return false;
	if(fields[6] == "1" || fields[6] == "2" || fields[6] == "3")
		_validFix = true;

	// convert utc seconds
	if(fields[1] == "")
	{
		_validFix = false;
		return false;
	}
	_utc_sec = atoi((fields[1].substr(0,2)).c_str())*3600;
	_utc_sec += atoi((fields[1].substr(2,2)).c_str())*60;
	_utc_sec += atoi((fields[1].substr(4,2)).c_str());

	// convert utc micro-seconds
	string tmp = "0.";
	tmp += fields[1].substr(6,3);
	_utc_sec += (int)rint(atof(tmp.c_str())*1000000.0);

	// convert lat
	if(fields[2] == "" || fields[3] == "")
	{
		_validFix = false;
		return false;
	}
	_lat = atof((fields[2].substr(0,2)).c_str());
	_lat += atof((fields[2].substr(2,7)).c_str())/60.0;
	if(fields[3] == "S")
		_lat *= (-1);

	// convert lng
	if(fields[4] == "" || fields[5] == "")
	{
		_validFix = false;
		return false;
	}
	_lng = atof((fields[4].substr(0,3)).c_str());
	_lng += atof((fields[4].substr(3,7)).c_str())/60.0;
	if(fields[5] == "W")
		_lng *= (-1);

	return true;
}

bool GPSEM406ASIRFIIISerial::validFix()
{
	return _validFix;
}

int GPSEM406ASIRFIIISerial::utc_sec()
{
	return _utc_sec;
}

int GPSEM406ASIRFIIISerial::utc_usec()
{
	return _utc_usec;
}

double GPSEM406ASIRFIIISerial::lat()
{
	return _lat;
}

double GPSEM406ASIRFIIISerial::lng()
{
	return _lng;
}

