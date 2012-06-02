#include <iostream>
#include <libplayerc++/playerc++.h>
#include <deque>
#include <math.h>
#include <algorithm>
#include <signal.h>
#include <fstream>
#include <iomanip>

#define _USE_MATH_DEFINES

using namespace PlayerCc;
using namespace std;

double headingError(double set, double meas);

int main(int argc, char *argv[])
{
try {
	char opt, way_opt;
	int stop = 0;
	int exit = 0;
	bool flash = 0;
	int counter, way_target, way_counter, way_start;
	double tol = 0.000005;
	ofstream writefile;
	ifstream readfile;
	string line;

	double tv, rv, set,error,meas, latitude, longitude, meas_latitude, meas_longitude, prev_error;
	double way_long, way_lat;
	int way_count = 0;
	double A = 0.8; // p
	double B = 0.0; // d
	double C = 0.01; // i
	double distance_center = 0.0; //distance from robot to object
	double distance_right = 0.0;
	double distance_left = 0.0;
	
	if (argc != 2)
	{
		cout << "Input file not specified" << endl;
		return -1;	
	}


	PlayerClient robot("localhost");
	Position2dProxy pp(&robot,0);
	PowerProxy wp(&robot,0);
	ImuProxy ii(&robot,0);
	SonarProxy test(&robot,0);
	//Status light proxy	
	DioProxy dp(&robot,0);
	//Estop proxy	
	DioProxy ep(&robot,1);	
	GpsProxy gp(&robot,0);

	robot.SetDataMode(PLAYER_DATAMODE_PULL);
	robot.SetReplaceRule(true, -1, -1, -1);


	cout.precision(15);
	cin.precision(15);

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
				cout << "o  ---  Set translational velocity and heading and avoid obstacles with sonar\n";
				cout << "g  ---  Navigate to GPS waypoint (wihtout avoidance\n";
				cout << "s  ---  Set velocities to zero\n";
				cout << "f  ---  Toggle light flashing\n";
				cout << "p  ---  Print status\n";
				cout << "w  ---  Mark waypoints and/or navigate to them\n";
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
			case 'o':
				cout << "Enter translational velocity: " << endl;
				cin >> tv;
				cout << "\nEnter heading: " << endl;
				cin >> set;

				while(1)
				{
					robot.Read();
					distance_center=test[0];
					distance_left=test[1];
					distance_right=test[2];
					meas = ii.GetPose().pyaw;
					error = headingError(set, meas);
					rv = A*error;

					cout << "Heading: " << meas << endl;
		
					cout << "Error :" << error << endl;

					cout << "rv: " << rv << endl;

					//pp.SetSpeed(30,0);
					if(distance_center < 48.0 && distance_right > 80.0)
					{
						rv = 50;
					}
					else if(distance_center < 48.0 && distance_left > 80.0)
					{
						rv = -50;
					}


					if(distance_center < 18.0 || distance_right < 18.0 || distance_left < 18.0)
					{
						pp.SetSpeed(0,0);
					}
					else
					{
						pp.SetSpeed(tv,rv);	
					}
					
					cout << ep.GetInput(0) << endl;
					if(ep.GetInput(0) == 1)
					{
						cout << "E-Stop Condition" << endl;
						pp.SetSpeed(0,0);
						break;
					}
				}
				break;
			case 'g':
				cout << "Enter Latitude: ";
				cin >> latitude;
				cout << "Enter Longitude: ";
				cin >> longitude;
				cout << "Enter translational velocity: ";
				cin >> tv;
				while(1)
				{
					robot.Read();
					distance_center=test[0];
					distance_left=test[1];
					distance_right=test[2];
					meas = ii.GetPose().pyaw;
					meas_longitude = gp.GetLongitude();
					meas_latitude = gp.GetLatitude();
					error = headingError(atan2((longitude - meas_longitude),(latitude - meas_latitude)),meas);			
					//error = headingError(atan2((longitude - gp.GetLongitude())*cos(gp.GetLatitude()),(latitude - gp.GetLatitude())), ii.GetPose().pyaw);
					rv = A*error;

					if(distance_center < 18.0 || distance_right < 18.0 || distance_left < 18.0)
					{
						pp.SetSpeed(0,0);
					}
					else
					{
						pp.SetSpeed(tv,rv);	
					}
				

					if(ep.GetInput(0) == 1)
					{
						cout << "E-Stop Condition" << endl;
						pp.SetSpeed(0,0);
						break;
					}
					prev_error = error;
				}				
				
			case 's':
				pp.SetSpeed(0,0);
				cout << "Stopped";
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
				cout << "Lat, Lng: " << gp.GetLatitude() << "," << gp.GetLongitude() << endl;
				cout << "Battery Voltage: " << wp.GetCharge() << endl;
				cout << "Estop Status: " << ep.GetInput(0) << endl;
				break;
			case 'w':
				readfile.open(argv[1]);
				way_count = 0;
				while (getline(readfile, line))
				{
					way_count++;
				}
				way_count = way_count/2;
				readfile.close();
				cout << "There are currently " << way_count << " waypoints stored" << endl;
				cout << "Enter w to store a waypoint, l to list waypoints, n to navigate to a waypoint, m to navigate all waypoints, c to clear waypoint file" << endl;
				cin >> way_opt;
				robot.Read();
				meas = ii.GetPose().pyaw;
				meas_longitude = gp.GetLongitude();
				meas_latitude = gp.GetLatitude();
				if (way_opt == 'w')
				{
					writefile.open(argv[1], ofstream::app);
					writefile << setprecision(15) << meas_longitude << endl;
					writefile << setprecision(15) << meas_latitude << endl;
					writefile.close();
					//way_long[way_count] = meas_longitude;
					//way_lat[way_count] = meas_latitude;
					//way_count++;
					break;			
				}
				else if (way_opt == 'l')
				{	
					counter = 0;					
					if (way_count == 0)
					{
						cout << "No waypoints currently stored" << endl;
						break;
					}
					readfile.open(argv[1]);
					while (1)
					{
						getline (readfile, line);
						if (!readfile.good()) break;
						cout << "Waypoint " << counter << endl;
						cout << "Longitude: " << setprecision(15) << line << endl;
						getline (readfile, line);
						cout << "Latitude: " << setprecision(15) << line << endl;					
						counter++;
					}
					readfile.close();
					break;
				}
				else if (way_opt == 'n')
				{
					cout << "way_count: " << way_count << endl;
					cout << "Enter the waypoint to navigate to: " << endl;
					cin >> way_target;
					cout << "Enter translational velocity: " << endl;
					cin >> tv;
					while (way_target > way_count)
					{
						cout << "Waypoint does not exist, re-enter waypoint number: " << endl;
						cin >> way_target;
					}
					readfile.open(argv[1]);
					counter = 0;
					while (counter + 1 < way_count)
					{
					getline(readfile, line);
					getline(readfile, line);
					counter++;
					}
					readfile >> setprecision(15) >> way_long;
					readfile >> setprecision(15) >> way_lat;
					cout << "way_long: " << way_long << endl;
					cout << "way_lat: " << way_lat << endl;
					readfile.close();
					dp.SetOutput(1,1);					
					robot.Read();
					distance_center=test[0];
					distance_left=test[1];
					distance_right=test[2];
					meas = ii.GetPose().pyaw;
					meas_longitude = gp.GetLongitude();
					meas_latitude = gp.GetLatitude();
					cout << "Longitude Error: " << abs(meas_longitude - way_long) << endl;
					cout << "Latitude Error: " << abs(meas_latitude - way_lat) << endl;
					cout << tol << endl;
					while((abs(meas_longitude - way_long) > tol) || (abs(meas_latitude - way_lat) > tol))
					{
						error = (180/M_PI)*(atan2(sin((M_PI/180)*(way_long-meas_longitude))*cos((M_PI/180)*way_lat), cos((M_PI/180)*meas_latitude)*sin((M_PI/180)*way_lat)-sin((M_PI/180)*meas_latitude)*cos((M_PI/180)*way_lat)*cos((M_PI/180)*(way_long-meas_longitude))));

						cout << "heading : " << fmod(error+360.0, 360.0) << endl;
						cout << "actual_heading: " << meas << endl;
						error = headingError(fmod(error + 360.0, 360.0),meas);
						cout << error << endl;
						rv = A * error + B * (error - prev_error);
						cout << distance_center << endl;
						cout << distance_right << endl;
						cout << distance_left << endl;
						if(distance_center < 18.0 || distance_right < 18.0 || distance_left < 18.0)
						{
							pp.SetSpeed(0,0);
						}
						else
						{
							pp.SetSpeed(tv,rv);	
						}
				
						robot.Read();
						if(ep.GetInput(0) == 1)
						{
							cout << "E-Stop Condition" << endl;
							pp.SetSpeed(0,0);
							break;
						}
						distance_center=test[0];
						distance_left=test[1];
						distance_right=test[2];
						meas = ii.GetPose().pyaw;
						meas_longitude = gp.GetLongitude();
						meas_latitude = gp.GetLatitude();
						cout << "Longitude Error: " << abs(meas_longitude - way_long) << endl;
						cout << "Latitude Error: " << abs(meas_latitude - way_lat) << endl;
					}
					cout << "Target reached" << endl;
					pp.SetSpeed(0,0);
					dp.SetOutput(1,0);
					break;
				}
				else if (way_opt == 'm')
				{	
					cout << "Enter translational velocity: " << endl;
					cin >> tv;
					cout << "Enter the starting waypoint to navigate to: " << endl;
					cin >> way_start;
					while (way_start >= way_count)
					{
						cout << "Waypoint does not exist, re-enter waypoint number: " << endl;
						cin >> way_start;
					}
					readfile.open(argv[1]);

					counter = 0;
					while (counter < way_start)
					{
					getline(readfile, line);
					getline(readfile, line);
					counter++;
					}

					way_counter = way_start;
					while (way_counter < way_count)
					{
						readfile >> setprecision(15) >> way_long;
						readfile >> setprecision(15) >> way_lat;
						//cout << "way_long: " << way_long << endl;
						//cout << "way_lat: " << way_lat << endl;
						dp.SetOutput(1,1);					
						robot.Read();
						distance_center=test[0];
						distance_left=test[1];
						distance_right=test[2];
						meas = ii.GetPose().pyaw;
						meas_longitude = gp.GetLongitude();
						meas_latitude = gp.GetLatitude();
						//cout << "Longitude Error: " << abs(meas_longitude - way_long) << endl;
						//cout << "Latitude Error: " << abs(meas_latitude - way_lat) << endl;
						//cout << tol << endl;
						cout << "Approaching Waypoint " << way_counter << endl;
						while((abs(meas_longitude - way_long) > tol) || (abs(meas_latitude - way_lat) > tol))
						{
							error = (180/M_PI)*(atan2(sin((M_PI/180)*(way_long-meas_longitude))*cos((M_PI/180)*way_lat), cos((M_PI/180)*meas_latitude)*sin((M_PI/180)*way_lat)-sin((M_PI/180)*meas_latitude)*cos((M_PI/180)*way_lat)*cos((M_PI/180)*(way_long-meas_longitude))));

							cout << "heading : " << fmod(error+360.0, 360.0) << endl;
							cout << "actual_heading: " << meas << endl;
							error = headingError(fmod(error + 360.0, 360.0),meas);
							//cout << error << endl;
							rv = A * error + B * (error - prev_error);
							//cout << distance_center << endl;
							//cout << distance_right << endl;
							//cout << distance_left << endl;
							if(distance_center < 18.0 || distance_right < 18.0 || distance_left < 18.0)
							{
								pp.SetSpeed(0,0);
							}
							else
							{
								pp.SetSpeed(tv,rv);	
							}
				
							robot.Read();
							if(ep.GetInput(0) == 1)
							{
								cout << "E-Stop Condition" << endl;
								pp.SetSpeed(0,0);
								break;
							}
							distance_center=test[0];
							distance_left=test[1];
							distance_right=test[2];
							meas = ii.GetPose().pyaw;
							meas_longitude = gp.GetLongitude();
							meas_latitude = gp.GetLatitude();
							//cout << "Longitude Error: " << abs(meas_longitude - way_long) << endl;
							//cout << "Latitude Error: " << abs(meas_latitude - way_lat) << endl;
						}
						if(ep.GetInput(0) == 1)
						{
							cout << "E-Stop Condition" << endl;
							pp.SetSpeed(0,0);
							readfile.close();
							break;
						}
						cout << "Waypoint " << way_counter << " reached" << endl;
						way_counter++;
					}
					readfile.close();
					pp.SetSpeed(0,0);
					dp.SetOutput(1,0);
					break;
				}
				else if (way_opt == 'c')
				{
					writefile.open(argv[1], ios::trunc);
					writefile.close();
					break;				
				}
			case 'q':
				cout << "Exit\n";
				exit = 1;
				break;
			}
	}
	
}
	catch (PlayerError e)
	{
		cerr << e << endl;
		return -1;
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




