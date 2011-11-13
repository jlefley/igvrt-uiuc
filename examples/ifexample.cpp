#include <iostream>
#include <stdlib.h>

using namespace std;

int main()
{
int leftSensorActive;
int rightSensorActive;
bool turnLeft=0;
bool turnRight=0;
bool goStraight=0;
bool stop=0;

cout << "Enter Left Sensor Active value(1 or 0); " << endl;
cin >> leftSensorActive;
cout << "Enter Right Sensor Active value(1 or 0); " << endl;
cin >>  rightSensorActive;

//Make a set of conditional statements to set the values of the boolean variables based on the sensor values

if(leftSensorActive == 0 && rightSensorActive == 0)
	{
	goStraight = 1;
	}
if(leftSensorActive == 1 && rightSensorActive == 0)
	{
	turnleft = 1;
	}
if(leftSensorActive == 0 && rightSensorActive == 1)
	{
	turnRight = 1;
	}
if(leftSensorActive == 1 && rightSensoractive == 1)
	{
	stop = 1;
	}

cout << "rightSensorActive =: " << rightSensorActive << endl;
cout << "leftSensorActive =: " << leftSensorActive << endl;
cout << "turnLeft =: " << turnLeft << endl;
cout << "turnRight =: " << turnRight << endl;
cout << "goStraight =: " << goStraight << endl;
cout << "stop =: " << stop << endl;

return 0;
}
