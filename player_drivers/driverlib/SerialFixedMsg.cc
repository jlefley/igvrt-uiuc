/*
 *  SerialFixedMsg.cpp
 *
 */

#include "SerialFixedMsg.h"

SerialFixedMsg::SerialFixedMsg() : data(1) {}

SerialFixedMsg::SerialFixedMsg(unsigned int numBytes) : data(numBytes) {}

void SerialFixedMsg::reset()
{
	for(unsigned int i = 0; i < Len(); i++)
	{
		data[i] = 0;
	}
}

const vector<unsigned char>& SerialFixedMsg::Data() const
{
	return data;
}

unsigned int SerialFixedMsg::Len() const
{
	return data.size();
}

string SerialFixedMsg::DataBits() const
{
	string s = "";
	
	for(unsigned int i = 0; i < Len(); i++)
	{
		s += ByteBits(data[i]) + " ";
	}
	
	return s;
}

string SerialFixedMsg::ByteBits(unsigned char c) const
{
	string s = "";
	
	if(c & 0x80)
		s += "1";
	else
		s += "0";
	
	if(c & 0x40)
		s += "1";
	else
		s += "0";
		
	if(c & 0x20)
		s += "1";
	else
		s += "0";
	
	if(c & 0x10)
		s += "1";
	else
		s += "0";
		
	if(c & 0x08)
		s += "1";
	else
		s += "0";
		
	if(c & 0x04)
		s += "1";
	else
		s += "0";
		
	if(c & 0x02)
		s += "1";
	else
		s += "0";
	
	if(c & 0x01)
		s += "1";
	else
		s += "0";
		
	return s;
}


