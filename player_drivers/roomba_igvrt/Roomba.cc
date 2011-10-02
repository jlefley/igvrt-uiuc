/*
 *  Roomba.cc
 *  roomba
 *
 */

#include "Roomba.h"

bool Roomba::wakeUp()
{
	if(!sendStr("$$$"))
		return false;
	usleep(1000000);
	
	if(!sendStr("S@,8080\n"))
		return false;
	usleep(1000000);
	
	if(!sendStr("S&,8080\n"))
		return false;
	usleep(1000000);
	
	if(!sendStr("S&,8000\n"))
		return false;
	usleep(1000000);
	
	if(!sendStr("---\n"))
		return false;
	usleep(2000000);

	return true;
}

bool Roomba::changeMode(eMode mode)
{
	switch(mode)
	{
	case OFF:
		if(!issueNoRespCmd(new CmdStart))
			return false;
		usleep(100000);
		if(!issueNoRespCmd(new CmdControl))
			return false;
		usleep(100000);
		if(!issueNoRespCmd(new CmdPower))
			return false;
		usleep(100000);
		return true;
		break;
	case PASSIVE:
		if(!issueNoRespCmd(new CmdStart))
			return false;
		usleep(100000);
		return true;
		break;
	case SAFE:
		if(!issueNoRespCmd(new CmdStart))
			return false;
		usleep(100000);
		if(!issueNoRespCmd(new CmdControl))
			return false;
		usleep(100000);
		if(!issueNoRespCmd(new CmdSafe))
			return false;
		usleep(100000);
		return true;
		break;
	case FULL:
		if(!issueNoRespCmd(new CmdStart))
			return false;
		usleep(100000);
		if(!issueNoRespCmd(new CmdControl))
			return false;
		usleep(100000);
		if(!issueNoRespCmd(new CmdFull))
			return false;
		usleep(100000);
		return true;
		break;
	}

	return false;
}

bool Roomba::changeBaud(unsigned int baudCode)
{
	if(!issueNoRespCmd(new CmdBaud(baudCode)))
		return false;
	usleep(1000000);
	return true;
}

