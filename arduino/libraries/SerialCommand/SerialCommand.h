#ifndef SERIALCOMMAND_H
#define SERIALCOMMAND_H

#include <inttypes.h>
#include "TimeOut.h"

#define SC_MAXDATA 16
#define SC_TIMEOUT_MS 1000


class SerialCommand
{
public:
	
	SerialCommand();
	
    // methods	
	void initialize(int baud, unsigned int (*decode)(unsigned char cmd), void (*execute)(unsigned char cmd, unsigned char * data));
	bool process();
	
private:
	
	enum eSerialState { IDLE, WAITING };
	
	// private members
	TimeOut timeOut;				// object to keep track of timeouts
	unsigned char msg[SC_MAXDATA];	// data for msg; length of max msg data
	
	unsigned char inByte;			// incoming serial byte
	eSerialState state;				// state of serial reader
	int bytesExpected;				// number of bytes to wait for
	unsigned int msgInd;			// index into serialMsg while reading msg
	unsigned char cmdToExecute;		// cmd to execute
	
	// private methods
	void waitForBytes(unsigned int n);
	void execute();
	
	// callbacks
	unsigned int (*decodeCallback)(unsigned char cmd);
	void (*executeCallback)(unsigned char cmd, unsigned char * data);
	
};

extern SerialCommand SerialCmd;

#endif
