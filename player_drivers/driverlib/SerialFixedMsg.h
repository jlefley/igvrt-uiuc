/*
 *  SerialFixedMsg.h
 *
 */

#ifndef SERIALFIXEDMSG_H
#define SERIALFIXEDMSG_H

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;

class SerialFixedMsg
{
public:
	SerialFixedMsg();
	SerialFixedMsg(unsigned int numBytes);
	
	const vector<unsigned char>& Data() const;
	unsigned int Len() const;
	string DataBits() const;
	virtual string PrettyData() const = 0;
	
protected:
	vector<unsigned char> data;

	string ByteBits(unsigned char c) const;
	void reset();

};

#endif
