/*
 *  SerialDevice.cc
 *  Serial Device
 *
 */

#include "SerialDevice.h"

SerialDevice::SerialDevice()
{
	fd = -1;
}

bool SerialDevice::connect(string serialPortName, int baudRate)
{
	fd = open(serialPortName.c_str(), O_RDWR | O_NOCTTY);

	if(fd < 0)
	{
		return false;
	}

	flush();

	//save current termios structure
	tcgetattr(fd, &oldtio);

	//setup new termios structure
	setupTermios(baudRate);

	return true;
}

bool SerialDevice::closeConnection()
{
	flush();

	//restore old termios structure
	tcsetattr(fd, TCSANOW, &oldtio);

	close(fd);

	return true;
}

void SerialDevice::flush()
{
	tcflush(fd, TCIFLUSH);
}

void SerialDevice::setupTermios(int baudRate)
{
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	cfsetspeed(&tio, baudRate);

	flush();

	tcsetattr(fd, TCSANOW, &tio);
}

bool SerialDevice::sendStr(const string & str)
{
	return (write(fd, str.c_str(), str.length()) >= 0);
}

bool SerialDevice::sendByte(const unsigned char c)
{
	return (write(fd, &c, 1) >= 0);
}

bool SerialDevice::sendBytes(const vector<unsigned char> & data)
{
	bool success = false;
	unsigned char * arr = new unsigned char[data.size()];

	for(unsigned int i=0; i < data.size(); i++)
	{
		arr[i] = data[i];
	}

	if(write(fd, arr, data.size()) == (int)data.size())
		success = true;

	delete arr;

	return success;
}

bool SerialDevice::recvBytes(vector<unsigned char> & data)
{
	unsigned char c;
	int numRead;
	unsigned int i;
	int selectRet;
	fd_set set;
	struct timeval timeout;	

	if(data.size() == 0)
	{
		return false;
	}

	// set timeout to 0.5 second
	timeout.tv_sec = READ_TIME_OUT_SEC;
	timeout.tv_usec = READ_TIME_OUT_USEC;	

	i = 0;
	while(i < data.size())
	{
		FD_ZERO(&set);
		FD_SET(fd, &set);

		selectRet = select(FD_SETSIZE, &set, NULL, NULL, &timeout);

		// select had an error
		if(selectRet < 0)
		{
			// error was actually an interrupt -- try again
			if(errno == EINTR)
			{
				continue;
			}
			// some other error -- end read
			else
			{
				break;
			}
		}
		// timeout -- end read
		else if(selectRet == 0)
		{
			break;
		}

		numRead = read(fd, &c, 1);
			
		if(numRead == 1)
		{
			data[i] = c;
			i++;
		}
		else if(numRead == 0)
		{
			
		}
		else
		{
			break;
		}
	}
	
	if(i == data.size())
	{
		return true;
	}
	

	return false;
}

bool SerialDevice::issueNoRespCmd(const SerialFixedMsg * m)
{
	bool success = false;

	if(m != NULL)
	{
		flush();

		success = sendBytes(m->Data());

		delete m;
	}

	return success;
}

bool SerialDevice::issueCmdWithResp(const SerialFixedMsg * m, SerialFixedResp & r)
{
	if(m == NULL)
	{
		return false;
	}

	if(issueNoRespCmd(m))
	{
		vector<unsigned char> data(r.Len());
	
		if(recvBytes(data))
		{
			r.LoadData(data);

			return true;
		}
	}

	return false;
}


