/*
 *  MotorControllerSerial.cc
 *  MotorControllerSerial
 *
 */

#include "MotorControllerSerial.h"

bool MotorControllerSerial::recvSensorData(SerialFixedResp & r)
{
	vector<unsigned char> header(1);
	vector<unsigned char> body(6);

	if(!recvBytes(header))
		return false;

	if(header[0] != 0xFF)
		return false;

	if(!recvBytes(body))
		return false;

	r.LoadData(body);

	return true;
}


