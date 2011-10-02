/*
 *  IRSonarProtoDev.h
 *  IR/Sonar Prototype
 *
 */

#ifndef IRSONARPROTODEV_H
#define IRSONARPROTODEV_H

//parent class
#include "../driverlib/SerialFixedMsg.h"
#include "../driverlib/SerialFixedResp.h"
#include "../driverlib/SerialDevice.h"

//c++ libs
#include <string>


using namespace std;

class IRSonarProtoDev : public SerialDevice
{
public:
	bool getData(int & ir, int & sonar);
};


#endif

