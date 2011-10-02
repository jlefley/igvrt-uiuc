
#include "WProgram.h"
#include "TimeOut.h"

TimeOut::TimeOut()
{
	clear();
}

void TimeOut::set(unsigned long ms)
{
	start = millis();
	diff = ms;
}

bool TimeOut::expired()
{
	if(diff > 0)
	{
		unsigned long curr = millis();
		
		if(curr < start || curr >= (start + diff))
		{
			clear();
			return true;
		}
	}
	
	return false;
}

void TimeOut::clear()
{
	diff = 0;
	start = 0;
}
