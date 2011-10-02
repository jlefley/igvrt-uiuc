/*
 *  SerialFixedResp.h
 *
 */

#ifndef ROOMBARESP_H
#define ROOMBARESP_H

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "SerialFixedMsg.h"

using namespace std;

class SerialFixedResp : public SerialFixedMsg
{
public:
	SerialFixedResp() : SerialFixedMsg() { reset(); }
	SerialFixedResp(unsigned int numBytes) : SerialFixedMsg(numBytes) { reset(); }

	void LoadData(const vector<unsigned char> & newData)
	{
		if(Len() != newData.size())
			throw runtime_error("SerialFixedResp: length of new data does not match msg size");

		for(unsigned int i = 0; i < Len(); i++)
			data[i] = newData[i];
	}
	
};

#endif
