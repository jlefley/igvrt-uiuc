#ifndef TIMEOUT_H
#define TIMEOUT_H

#include <inttypes.h>

class TimeOut
{
public:
	
	TimeOut();
	
    // methods	
	void set(unsigned long ms);
	bool expired();
	void clear();
	
private:
	
	// private members
	unsigned long start;	// timer start time
	unsigned long diff;		// ms until timeout

};

#endif
