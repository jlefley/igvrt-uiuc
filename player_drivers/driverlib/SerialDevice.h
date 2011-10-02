/*
 *  SerialDevice.h
 *  Serial Device
 *
 */

#ifndef SERIALDEVICE_H
#define SERIALDEVICE_H

#include "SerialFixedMsg.h"
#include "SerialFixedResp.h"

//c++ libs
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

//c libs
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

using namespace std;

class SerialDevice
{
public:

	static const double READ_TIME_OUT_SEC = 2; // number of seconds before time out
	static const double READ_TIME_OUT_USEC = 0; // number of microseconds before time out (total time out = sec + usec)

	SerialDevice();

	bool connect(string serialPortName, int baudRate);
	bool closeConnection();
	void flush();

	bool issueNoRespCmd(const SerialFixedMsg * m);
	bool issueCmdWithResp(const SerialFixedMsg * m, SerialFixedResp & r);

protected:
	int fd;
	struct termios tio;
	struct termios oldtio;
	
	void setupTermios(int baudRate);

	bool sendStr(const string & str);
	bool sendByte(const unsigned char c);
	bool sendBytes(const vector<unsigned char> & data);
	bool recvBytes(vector<unsigned char> & data);
};

#endif

