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

	char opt;
	int stop = 0;
	int exit = 0;
	bool flash = 0;

	double tv, rv, set,error,meas;
	double A = -1; // p
	double B = 0.1; // d
	double C = 0.01; // i
	double distance = 0; //distance from robot to object

	PlayerClient robot("localhost");
	Position2dProxy pp(&robot,0);
	PowerProxy wp(&robot,0);
	ImuProxy ii(&robot,0);
	SonarProxy test(&robot,0);
	DioProxy dp(&robot,0);

	cout << "IGVRT Player Client\n";
	cout << "Enter h for a list of options\n";
	
	while (exit == 0)
	{
	cout << ">> ";
	cin >> opt;
		switch (opt)
			{
			case 'h':
				cout << "Command Options:\n";
				cout << "m  ---  Set translational, rotational velocity\n";
				cout << "l  ---  Set translational velocity, heading\n";
				cout << "o  ---  Set translational velocity and heading and avoid obstacles with sonar\n";
				cout << "s  ---  Set velocities to zero\n";
				cout << "c  ---  Check battery voltage\n";
				cout << "f  ---  Toggle light flashing\n";
				cout << "p  ---  Print sonar and IMU data\n";
				cout << "q  ---  Quit\n";
				break;
			case 'm':
				cout << "\nManual Mode\n";
				cout << "Enter translational velocity: ";
				cin >> tv;
				cout << "\nEnter rotational velocity: ";
				cin >> rv;
				cout << "\nStarting, press space to stop";
				pp.SetSpeed(tv,rv);
				break;
			case 'l':
				cout << "Enter translational velocity: ";
				cin >> tv;
				cout << "\nEnter heading: ";
				cin >> set;
				cout << "\nStarting...";

				while(1)
				{
					robot.Read();
					meas = ii.GetPose().pyaw;
					meas = 360;
					error = headingError(set, meas);
					rv = A*error;
					
					cout << "Heading: " << meas << endl;
		
					cout << "Error :" << error << endl;

					cout << "rv: " << rv << endl;

					pp.SetSpeed(tv,rv);
				}
			case 'o':
				cout << "Enter translational velocity: ";
				cin >> tv;
				cout << "\nEnter heading: ";
				cin >> set;
				cout << "\nStarting...";

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

					//pp.SetSpeed(30,0);
					if(distance<36.0 && distance>30.0)
					{
					pp.SetSpeed(15,30);
					}
					else if(distance<30.0 && distance>24.0)
					{
					pp.SetSpeed(10,40);
					}
				else if(distance<24.0 && distance>18.0)
					{
					pp.SetSpeed(5,60);
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
			case 's':
				pp.SetSpeed(0,0);
				cout << "Stopped";
				break;
			case 'c':
				robot.Read();
				cout << "X Speed: " << pp.GetXSpeed() << "\n";
				cout << "Y Speed: " << pp.GetYSpeed() << "\n";
				cout << "X Position: " << pp.GetXPos() << "\n";
				cout << "Y Position: " << pp.GetYPos() << "\n";
				cout << "Yaw: " << pp.GetYaw() << "\n";
				cout << "Battery Voltage: " << wp.GetCharge() << "\n";
				break;	
			case 'f':
				if(flash == true)
				{
				dp.SetOutput(1,0);
				flash = false;
				}
				else
				{
				dp.SetOutput(1,1);
				flash = true;
				}
				break;
			case 'p':
				robot.Read();
				cout << "Center distance: " << test[0] << endl;
				cout << "Left distance: " << test[1] << endl;
				cout << "Right distance: " << test[2] << endl;
				cout << "Heading: " << ii.GetPose().pyaw << endl;
				break;
			case 'q':
				cout << "Exit\n";
				exit = 1;
				break;
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




