/*
 *  IRSonarProtoDev.cc
 *  IR/Sonar Prototype
 *
 */

#include "IRSonarProtoDev.h"

bool IRSonarProtoDev::getData(int & ir, int & sonar)
{
	vector<unsigned char> sentinal(2);
	vector<unsigned char> data(4);
	bool sentinalFound = false;

	for(int i = 0; i < 20; i++)
	{
		// if recv fails, return false
		if(!recvBytes(sentinal))
			return false;

		if(sentinal[0] == 0xFF && sentinal[1] == 0xFF)
		{
			sentinalFound = true;
			break;
		}
	}

	if(!sentinalFound)
		return false;

	if(!recvBytes(data))
		return false;

	ir = (data[0] << 8) + data[1];
	sonar = (data[2] << 8) + data[3];

	return true;
}

