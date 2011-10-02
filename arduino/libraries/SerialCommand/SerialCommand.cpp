
#include "WProgram.h"
#include "SerialCommand.h"

SerialCommand SerialCmd;              // preinstatiate

SerialCommand::SerialCommand()
{
	inByte = 0;
	state = IDLE;
	bytesExpected = 0;
	msgInd = 0;
	cmdToExecute = 0;
}

void SerialCommand::initialize(int baud, unsigned int (*decode)(unsigned char cmd), void (*execute)(unsigned char cmd, unsigned char * data))
{
	Serial.begin(baud);
	decodeCallback = decode;
	executeCallback = execute;
}

void SerialCommand::waitForBytes(unsigned int n)
{
	if(n > 0)
	{
		if(n > SC_MAXDATA)
			bytesExpected = SC_MAXDATA;
		else
			bytesExpected = n;
		
		msgInd = 0;
		state = WAITING;
		timeOut.set(SC_TIMEOUT_MS);
	}
	else
	{
		execute();
	}
}

void SerialCommand::execute()
{
	timeOut.clear();
	executeCallback(cmdToExecute, msg);
	state = IDLE;
	bytesExpected = 0;
	cmdToExecute = 0;
}

bool SerialCommand::process()
{
	bool moreAvailable = false;
	
	if(timeOut.expired())
	{
		state = IDLE;
		//Serial.flush();
	}
	
	if(Serial.available() > 0)
	{
		inByte = Serial.read();
		
		switch(state)
		{
		case IDLE:
			
			cmdToExecute = inByte;
			waitForBytes(decodeCallback(inByte));
			moreAvailable = true;
			
			break;
				
		case WAITING:
				
			if(bytesExpected > 0)
			{
				bytesExpected--;
				msg[msgInd] = inByte;
				msgInd++;
				moreAvailable = true;
			}
			
			if(bytesExpected <= 0)
			{
				execute();
			}
			
			break;
		}
	}
	
	return moreAvailable;
	
}
