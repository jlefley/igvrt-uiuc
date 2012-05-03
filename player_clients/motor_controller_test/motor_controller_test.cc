#include <iostream>
#include <libplayerc++/playerc++.h>
#include <deque>
#include <math.h>
#include <algorithm>

using namespace PlayerCc;
using namespace std;

double headingError(double set, double meas);

int main(int argc, char *argv[])
{
	PlayerClient robot("localhost");
	Position2dProxy pp(&robot,0);
	ImuProxy ii(&robot,0);
	SonarProxy test(&robot,0);
	double tv, rv, set,error,meas;
	double A = -1; // p
	double B = 0.1; // d
	double C = 0.01; // i
	double distance=0; //distance from robot to object
	cout << "Enter heading: ";
	cin >> set;
	cout << "Enter tv: ";
	cin >> tv;
	
	
	while(1)
	{
		robot.Read();
		distance=test[0];
		meas = ii.GetPose().pyaw;
		error = headingError(set, meas);
		rv = A*error;

		cout << "Heading: " << meas << endl;
		
		cout << "Error :" << error << endl;

		cout << "rv: " << rv << endl;

		cout << "Range: " << distance << endl;
		//pp.SetSpeed(30,0);
		if(distance<100.0 && distance>50.0)
			{
			pp.SetSpeed(20,80);
			}
		else if(distance<50.0 && distance>24.0)
			{
			pp.SetSpeed(20,60);
			}
		else if(distance<24.0 && distance>18.0)
			{
			pp.SetSpeed(20,90);
			}
		else if(distance<18.0)
			{
			pp.SetSpeed(0,0);
			}
		else
			{
			pp.SetSpeed(tv,rv);
			}	
	}

}

double headingError(double set, double meas)
{
	double e0,e1,headingMag;
	bool overflow;
	double headingSign = 1.0;

	e0 = fabs(set - meas);
	e1 = 360 - e0;

	if(e0 < e1)
		headingMag = e0;
	else
		headingMag = e1;

	if(set >= 0 && set <= 180)
	{
		if(meas <= (set + 180) && meas >= set)
			headingSign = -1.0;
	}
	else
	{
		if((meas <= 360 && meas >= set) || (meas >= 0 && meas <= (set - 180)))
			headingSign = -1.0;
	}

	return headingMag*headingSign;
		
}




