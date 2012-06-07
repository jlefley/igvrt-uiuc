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

double headingError(double, double);
double constrain(double, double, double);
double getHeading(double, double, double, double);
double getDistance(double, double, double, double);
int sign(double val);

int main(int argc, char *argv[])
{
try {
	char opt, way_opt;
	int stop = 0;
	int exit = 0;
	bool flash = 0;
	int counter, way_target, way_counter, way_start;
	double tol = 0.000008;
	ofstream writefile;
	ifstream readfile;
	string line;

	double tv, rv, set, meas, latitude, longitude, meas_latitude, meas_longitude, target_heading;
	double error = 0;
	double prev_error = 0;
	double integrated_error = 0;
	double derivative_error = 0;
	double way_long, way_lat;
	int way_count = 0;
	double A = 0.4; // p
	double B = 0.02; // d
	double C = 0.000012; // i
	const double GUARD_GAIN = 1e5; //i control constraint
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
				cout << "n  ---  Monitor sonar\n";
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
					cout << "Error: " << error << endl;
					integrated_error += error;
					cout << "Integrated error: " << integrated_error << endl;
					derivative_error = error - prev_error;
					cout << "Differentiated error: " << derivative_error << endl;
					rv = A*error + B*derivative_error + C*constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);

					//cout << "Heading: " << meas << endl;
		
					//cout << "Error :" << error << endl;

					//cout << "rv: " << rv << endl;

					//pp.SetSpeed(30,0);
							if(distance_center < 36.0)
							{
								pp.SetSpeed(5,50);
								sleep(0.75);
								pp.SetSpeed(20,0);
							}						
							
							if(distance_center < 18.0)
							{
								pp.SetSpeed(-20,-50);
								sleep(1);
							}

					if(distance_center < 12.0 /*|| distance_right < 18.0 || distance_left < 18.0*/)
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
						target_heading = getHeading(meas_longitude, meas_latitude, way_long, way_lat);
						error = headingError(target_heading,meas);
						integrated_error += error;
						cout << "Integrated error: " << integrated_error << endl;
						derivative_error = error - prev_error;
						cout << "Differentiated error: " << derivative_error << endl;
						rv = A*error + B*derivative_error + C*constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);

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
							target_heading = getHeading(meas_longitude, meas_latitude, way_long, way_lat);
							error = headingError(target_heading,meas);
							cout << getDistance(meas_longitude, meas_latitude, way_long, way_lat) << endl;
							//cout << "Error: " << error << endl;
							integrated_error += error;
							//cout << "Integrated error: " << integrated_error << endl;
							derivative_error = error - prev_error;
							//cout << "Differentiated error: " << derivative_error << endl;
							rv = A*error + B*derivative_error + C*constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
							
							if(distance_center <= 15.0 || distance_right < 18.0 || distance_left < 18.0)
							{
								pp.SetSpeed(0,0);
							}
							else
							{
								/*if(rv > 2*tv)
								{
									pp.SetSpeed(tv/2, rv);
								}
								else
								{
									if(rv < -2*tv)
									{
										pp.SetSpeed(tv/2,rv);
									}
									else
									{
										pp.SetSpeed(tv,rv);
									}
								}*/

								if(rv > 30 && getDistance(meas_longitude, meas_latitude, way_long, way_lat) < 2.0)
								{
									pp.SetSpeed(5, rv);
								}
								else
								{
									if(rv < -30 && getDistance(meas_longitude, meas_latitude, way_long, way_lat) < 2.0)
									{
										pp.SetSpeed(5, rv);
									}
									else
									{
										pp.SetSpeed(tv, rv);
									}
								}
								if(distance_center < 60.0)
								{
									if(distance_right < 60.0)
									{
										//need to condition on boundaries
										pp.SetSpeed(tv, -80);
									}
									else if(distance_left < 60.0)
									{
										//need to condition on boundaries
										pp.SetSpeed(tv, 80);
									}
								}						
							
								if(distance_center < 18.0)
								{
									pp.SetSpeed(-40,0);
									sleep(1);
								}
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
					cout << "erasing" << endl;
					writefile.close();
					break;				
				}
			case 'n':
				while(1)
				{
					robot.Read();
					cout << "Center: " << test[0] << endl;
					cout << "Left: " << test[1] << endl;
					cout << "Right: " << test[2] << endl;
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

double constrain(double value, double min, double max)
{
	if(value > min)
	{
		if(value < max)
		{
			return value;
		}
		else
		{
			return max;
		}
	}
	else
	{
		return min;
	}
} 

double getHeading(double cur_lng, double cur_lat, double target_lng, double target_lat)
{

	double target, target_heading;

	target = (180/M_PI)*(atan2(sin((M_PI/180)*(target_lng-cur_lng))*cos((M_PI/180)*target_lat), cos((M_PI/180)*cur_lat)*sin((M_PI/180)*target_lat)-sin((M_PI/180)*cur_lat)*cos((M_PI/180)*target_lat)*cos((M_PI/180)*(target_lng-cur_lng))));

	target_heading = fmod(target + 360.0, 360.0);

	return target_heading;
}

double getDistance(double cur_lng, double cur_lat, double target_lng, double target_lat)
{
	return 6366692.07 * acos(sin((M_PI/180)*cur_lat) * sin((M_PI/180)*target_lat) + cos((M_PI/180)*cur_lat) * cos((M_PI/180)*target_lat) * cos((M_PI/180)*(cur_lng - target_lng)));

}

int sign(double val)
{
	if(val > 0.0)
	{
		return 1;
	}
	else
	{
		if(val < 0.0)
		{
			return -1;
		}
		else
		{
			return 0;
		}
	}
}
