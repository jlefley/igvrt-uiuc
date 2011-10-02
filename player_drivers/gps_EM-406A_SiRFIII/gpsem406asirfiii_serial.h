/*
 *  gpsem406asirfiii_serial.h
 *  gpsem406asirfiii_serial
 *
 */

#ifndef GPSEM406ASIRFIIISERIAL_H
#define GPSEM406ASIRFIIISERIAL_H

//parent class
#include "../driverlib/SerialFixedMsg.h"
#include "../driverlib/SerialFixedResp.h"
#include "../driverlib/SerialDevice.h"

//c++ libs
#include <string>
#include <vector>

//c libs
#include <math.h>


using namespace std;

class GPSEM406ASIRFIIISerial : public SerialDevice
{
public:
	GPSEM406ASIRFIIISerial();

	bool recvData();
	
	bool validFix();
	int utc_sec();
	int utc_usec();
	double lat();
	double lng();

protected:

	bool _validFix;
	int _utc_sec;
	int _utc_usec;
	double _lat;
	double _lng;

	bool recvMsg(string & msg);
	bool parseMsg(const string & msg, vector<string> & fields);
	bool saveData(const vector<string> & fields);
	bool saveData_GPGGA(const vector<string> & fields);

};

#endif

