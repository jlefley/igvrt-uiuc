#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <math.h>
#include <deque>
#include <algorithm>
#include <signal.h>
#include <fstream>
#include <iomanip>

#define _USE_MATH_DEFINES

//for opencv c
#include <stdio.h>
#include <cv.h>
#include <highgui.h>

using namespace PlayerCc;
using namespace std;
using namespace cv;

///////////////////////////////////////////
//Player client
double headingError(double, double);
double constrain(double, double, double);
double getHeading(double, double, double, double);
double getDistance(double, double, double, double);
int sign(double val);
///////////////////////////////////////////

///////////////////////////////////////////
//Image processing
int ImageProcess();

int thresh = 125; 
int thresh2 = 125;
int cutoff = 1000;

Mat leftim, rightim, trans_left, trans_right, frame_trans_left, frame_trans_right;
vector<Mat> planes_left;
vector<Mat> planes_right;
Mat lines_left;
Mat lines_right;
Mat cont_left;
Mat cont_right;
vector<vector<Point> > countours_left, countours_right;
vector<vector<Point> >::iterator it, it2;
vector<CvPoint>::iterator it3;
vector<Point> a, b;

Point2f src_left[4], src_right[4], dst_left[4], dst_right[4];
int pt_ind_left = 0;
int pt_ind_right = 0;

int x_left = 100;
int x_right = 100;

int flag_left = 0;
int flag_right = 0;

double cal_dist_left, cal_dist_right; // Pixel distance coresponding to cal_dist ft
double cal_dist = 1.6667;

//SENSOR DATA
int UpdateVars();
double real_dist_left, real_dist_right;
double distance_center = 0.0; //distance from robot to object
double distance_right = 0.0;
double distance_left = 0.0;
bool obstacle_left = false;
bool obstacle_right = false;
bool obstacle_center = false;
double line_error = 0;
double prev_line_error = 0;
int estop = 0;
double meas = 0.0;
double meas_latitude = 0.0;
double meas_longitude = 0.0;
double charge = 0.0;
	

double df(int x1, int y1, int x2, int y2);
///////////////////////////////////////////


int main(int argc, char *argv[])
{

boost::thread t(&ImageProcess);
boost::thread t2(&UpdateVars);
sleep(5);

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

	double tv, rv, set, latitude, longitude, target_heading;
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
	double thresh_react = 10.0;

	if (argc != 2)
	{
		cout << "Input file not specified" << endl;
		return -1;	
	}


	PlayerClient robot("localhost");
	Position2dProxy pp(&robot,0);
	//PowerProxy wp(&robot,0);
	//ImuProxy ii(&robot,0);
	//SonarProxy test(&robot,0);
	//Status light proxy	
	DioProxy dp(&robot,0);
	//Estop proxy	
	//DioProxy ep(&robot,1);	
	//GpsProxy gp(&robot,0);

	//robot.SetDataMode(PLAYER_DATAMODE_PULL);
	//robot.SetReplaceRule(true, -1, -1, -1);


	cout.precision(15);
	cin.precision(15);

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
				cout << "s  ---  Set velocities to zero\n";
				cout << "f  ---  Toggle light flashing\n";
				cout << "p  ---  Print status\n";
				cout << "w  ---  Mark waypoints and/or navigate to them\n";
				cout << "n  ---  Monitor sonar\n";
				cout << "l  ---  Lane Following\n";
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
	
					if(estop == 1)
					{
						cout << "E-Stop Condition" << endl;
						pp.SetSpeed(0,0);
						break;
					}
					prev_error = error;
				}
				break;
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
				cout << "Center distance: " << distance_center << endl;
				cout << "Left distance: " << distance_left << endl;
				cout << "Right distance: " << distance_right << endl;
				cout << "Heading: " << meas<< endl;
				cout << "Lat, Lng: " << meas_latitude << "," << meas_longitude << endl;
				cout << "Battery Voltage: " << charge << endl;
				cout << "Estop Status: " << estop << endl;
				cout << "Left clearance: " << real_dist_left << endl;
				cout << "Right clearance: " << real_dist_right << endl;
				cout << "Obstacle Left: " << obstacle_left << endl;
				cout << "Obstacle Right: " << obstacle_right << endl;
				cout << "Obstacle Center: " << obstacle_center << endl;
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
				cout << "Enter w to store a waypoint, l to list waypoints, m to navigate all waypoints, c to clear waypoint file" << endl;
				cin >> way_opt;
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
				else if (way_opt == 'm')
				{	
					cout << "Enter translational velocity: " << endl;
					cin >> tv;
					cout << "Enter the starting waypoint to navigate to: " << endl;
					cin >> way_start;
					dp.SetOutput(1,1);
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
						//cout << "Longitude Error: " << abs(meas_longitude - way_long) << endl;
						//cout << "Latitude Error: " << abs(meas_latitude - way_lat) << endl;
						//cout << tol << endl;
						cout << "Approaching Waypoint " << way_counter << endl;
						while((abs(meas_longitude - way_long) > tol) || (abs(meas_latitude - way_lat) > tol))
						{
							//cout << real_dist_left << " " << real_dist_right << endl;
							target_heading = getHeading(meas_longitude, meas_latitude, way_long, way_lat);
							error = headingError(target_heading,meas);
							//cout << getDistance(meas_longitude, meas_latitude, way_long, way_lat) << endl;
							//cout << "Error: " << error << endl;
							integrated_error += error;
							//cout << "Integrated error: " << integrated_error << endl;
							derivative_error = error - prev_error;
							//cout << "Differentiated error: " << derivative_error << endl;
							rv = A*error + B*derivative_error + C*constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
							line_error = 8 * (real_dist_right - real_dist_left) + 0.1 * (line_error - prev_line_error);
							
							if(distance_center <= 15.0 || distance_right < 18.0 || distance_left < 18.0)
							{
								pp.SetSpeed(0,0);
							}
							else
							{
								///WAYPOINT NAVIGATION: LOWEST IMPORTANCE IN HEIRARCHICAL CONTROL STRUCTURE 
								///(SEE CASE "NO OBSTACLES, DO NOTHING UNLESS YOU SEE LINES" BELOW)
								//OBSTACLE AVOIDANCE AND LANE FOLLOWING: EQUAL (GREATER THAN WAYPOINTS) IMPORTANCE
								//IN CONTROL STRUCTURE

								if(obstacle_center)
								{
									if(obstacle_right)
									{
										if(obstacle_left)
										{
											//OBSTACLES LEFT, RIGHT, CENTER
											cout << "LRC" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{
												pp.SetSpeed(15,50);
											}
											else
											{
												if(line_error > thresh_react)
												{			
													//CLOSER TO THE LEFT
													pp.SetSpeed(15, 50);
												}
												if(line_error < -thresh_react)
												{
													//CLOSER TO THE RIGHT
													//pp.SetSpeed(15, -50);
													pp.SetSpeed(15,50);
												}
												if(line_error <= thresh_react && line_error >= -thresh_react)
												{
													pp.SetSpeed(15, 50);
												}
											}
										}
										else
										{
											//OBSTACLES RIGHT, CENTER
											cout << "RC" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{
												pp.SetSpeed(15, -50);
											}
											else
											{
												if(line_error > thresh_react)
												{			
													//CLOSER TO THE LEFT
													//pp.SetSpeed(15,-30);
													pp.SetSpeed(15, -50);
												}
												if(line_error < -thresh_react)
												{
													//CLOSER TO THE RIGHT
													pp.SetSpeed(15,-50);
												}
												if(line_error >= -thresh_react && line_error<=thresh_react)
												{
													pp.SetSpeed(15,-50);
												}
											}
										}
									}
									else
									{
										if(obstacle_left)
										{
											//OBSTACLES LEFT, CENTER
											cout << "LC" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{
												pp.SetSpeed(15,50);
											}
											else
											{
												if(line_error > thresh_react)
												{			
													//CLOSER TO THE LEFT
													pp.SetSpeed(15,50);
												}
												if(line_error < -thresh_react)
												{
													//CLOSER TO THE RIGHT
													//pp.SetSpeed(15,30);
													pp.SetSpeed(15,50);
												}
												if(line_error >= -thresh_react && line_error <= thresh_react)
												{
													pp.SetSpeed(15,50);
												}
											}
										}
										else
										{
											//OBSTACLES CENTER
											cout << "C" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{
												pp.SetSpeed(15,50);	
											}
											else
											{
												if(line_error > thresh_react)
												{			
													//CLOSER TO THE LEFT
													pp.SetSpeed(15, 50);
												}
												if(line_error < -thresh_react)
												{
													//CLOSER TO THE RIGHT
													pp.SetSpeed(15, -50);
													pp.SetSpeed(15,50);
												}
												if(line_error <= thresh_react && line_error >= thresh_react)
												{
													pp.SetSpeed(15, 50);
												}
											}
										}
									}
								}
								else
								{
									if(obstacle_right)
									{
										if(obstacle_left)
										{
											//OBSTACLES LEFT, RIGHT
											cout << "LR" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{
												pp.SetSpeed(20,10);
											}
											else
											{
												if(line_error > thresh_react)
												{
													//CLOSER TO THE LEFT
													pp.SetSpeed(20, 10);
												}
												if(line_error < -thresh_react)
												{
													//CLOSER TO THE RIGHT
													pp.SetSpeed(20, 10);
												}
												if(line_error <= thresh_react && line_error >= -thresh_react)
												{
													pp.SetSpeed(20, 10);
												}
											}
										}
										else
										{
											//OBSTACLES RIGHT
											cout << "R" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{
												pp.SetSpeed(15,-50);
											}
											else
											{
												if(line_error > thresh_react)
												{			
													cout << "a" << endl;
													//CLOSER TO THE LEFT
													//pp.SetSpeed(15,50);
													pp.SetSpeed(15,-50);
												}
												if(line_error < -thresh_react)
												{
													cout << "b" << endl;
													//CLOSER TO THE RIGHT
													pp.SetSpeed(15,-50);
												}
												if(line_error <= thresh_react && line_error > -thresh_react)
												{
													cout << "c" << endl;
													pp.SetSpeed(15,-50);
												}
											}
										}
									}
									else
									{
										if(obstacle_left)
										{
											//OBSTACLES LEFT
											cout << "L" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{
												pp.SetSpeed(15,50);
											}
											else
											{	
												if(line_error > thresh_react)
												{			
													//CLOSER TO THE LEFT
													pp.SetSpeed(15, 50);
												}
												if(line_error < -thresh_react)
												{
													//CLOSER TO THE RIGHT
													//pp.SetSpeed(15, -50);
													pp.SetSpeed(15,50);
												}
												if(line_error < thresh_react && line_error > -thresh_react)
												{
													pp.SetSpeed(15, 50);
												}
											}
										}
										else
										{
											//NO OBSTACLES, DO NOTHING UNLESS YOU SEE LINES
											cout << "NO OBSTACLES" << endl;
											if(real_dist_right > 6.8 && real_dist_left > 6.8)
											{				
												if(rv > 30 && getDistance(meas_longitude, meas_latitude, way_long, way_lat) < 3.0)
												{
													pp.SetSpeed(15, rv);
												}
												else
												{
													if(rv < -30 && getDistance(meas_longitude, meas_latitude, way_long, way_lat) < 3.0)
													{
														pp.SetSpeed(15, rv);
													}
													else
													{
														pp.SetSpeed(tv, rv);
													}
												}
											}
											else
											{
											//AVOID LINES
											cout << "LINES" << endl;
											pp.SetSpeed(tv, line_error);
											}
										}
									}
								}

								//NEED TO CONDITION ON GETTING TOO CLOSE TO AN OBSTACLE
								/*								
								if(distance_center < 18.0)
								{
									pp.SetSpeed(-40,0);
									sleep(2);
								}
								if(distance_center < 18.0)
								{
									pp.SetSpeed(-40,0);
									sleep(2);
								}
								if(distance_center < 18.0)
								{
									pp.SetSpeed(-40,0);
									sleep(2);
								}
								*/
							}

							if(estop == 1)
							{
								cout << "E-Stop Condition" << endl;
								pp.SetSpeed(0,0);
								break;
							}
							prev_error = error;
							prev_line_error = line_error;
							//cout << "Longitude Error: " << abs(meas_longitude - way_long) << endl;
							//cout << "Latitude Error: " << abs(meas_latitude - way_lat) << endl;
						}
						if(estop == 1)
						{
							cout << "E-Stop Condition" << endl;
							pp.SetSpeed(0,0);
							dp.SetOutput(1,0);
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
					cout << "Center: " << distance_center<< endl;
					cout << "Left: " << distance_left << endl;
					cout << "Right: " << distance_right << endl;
				}
				break;
			case 'l':
				cout << "Enter translational velocity: " << endl;
				cin >> tv;
				dp.SetOutput(1,1);
				while(1)
				{
					line_error = 8 * (real_dist_right - real_dist_left) + 0.1 * (line_error - prev_line_error);
					if(real_dist_right > 6.8 && real_dist_left > 6.8)
					{						
						pp.SetSpeed(tv,0);
					}
					else
					{
						pp.SetSpeed(tv, line_error);

					}
					cout << line_error << endl;
					

					if(estop == 1)
						{
							cout << "E-Stop Condition" << endl;
							pp.SetSpeed(0,0);
							dp.SetOutput(1,0);
							break;
						}
					prev_line_error = line_error;
				}
				break;
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

int UpdateVars()
{
	try
	{
		PlayerClient robot1("localhost");
		//Position2dProxy pp(&robot1,0);
		PowerProxy wp(&robot1,0);
		ImuProxy ii(&robot1,0);
		SonarProxy test(&robot1,0);
		//Status light proxy	
		//DioProxy dp(&robot1,0);
		//Estop proxy	
		DioProxy ep(&robot1,1);	
		GpsProxy gp(&robot1,0);

		robot1.SetDataMode(PLAYER_DATAMODE_PULL);
		robot1.SetReplaceRule(true, -1, -1, -1);

		while(1)
		{
			robot1.Read();
			distance_center=test[0];
			distance_left=test[1];
			distance_right=test[2];
			meas = ii.GetPose().pyaw;
			meas_longitude = gp.GetLongitude();
			meas_latitude = gp.GetLatitude();
			charge = wp.GetCharge();

			if(ep.GetInput(0) == 1)
			{
			estop = 1;
			}
			else
			{
			estop = 0;
			}

			if(distance_center < 36)
			{
				obstacle_center = true;
			}
			else
			{
				obstacle_center = false;
			}

			if(distance_left < 36)
			{
				obstacle_left = true;
			}
			else
			{
				obstacle_left = false;
			}


			if(distance_right < 36)
			{
				obstacle_right = true;
			}
			else
			{
				obstacle_right = false;
			}
		}
	}
	catch (PlayerError e)
	{
		cerr << e << endl;
		return -1;
	}
}

int ImageProcess()
{
	ifstream readfile2;
	string line2;

	namedWindow("Left",1);
	namedWindow("Right",1);
	namedWindow("Composite Left",1);
	namedWindow("Composite Right",1);
	namedWindow("Calibrate Left", 1);
	namedWindow("Calibrate Right", 1);

	int counter = 0;
	int counter2 = 0;
	int dist_left = 0;
	int dist_right = 0;

	readfile2.open("cal_data.txt");

	readfile2 >> setprecision(15) >> src_left[0].x;
	readfile2 >> setprecision(15) >> src_left[0].y;
	readfile2 >> setprecision(15) >> src_left[1].x;
	readfile2 >> setprecision(15) >> src_left[1].y;
	readfile2 >> setprecision(15) >> src_left[2].x;
	readfile2 >> setprecision(15) >> src_left[2].y;
	readfile2 >> setprecision(15) >> src_left[3].x;
	readfile2 >> setprecision(15) >> src_left[3].y;	

	readfile2 >> setprecision(15) >> src_right[0].x;
	readfile2 >> setprecision(15) >> src_right[0].y;
	readfile2 >> setprecision(15) >> src_right[1].x;
	readfile2 >> setprecision(15) >> src_right[1].y;
	readfile2 >> setprecision(15) >> src_right[2].x;
	readfile2 >> setprecision(15) >> src_right[2].y;
	readfile2 >> setprecision(15) >> src_right[3].x;
	readfile2 >> setprecision(15) >> src_right[3].y;

	readfile2.close();

	VideoCapture cap2(2); //2
	VideoCapture cap3(3); //3

	if(!cap2.isOpened() || !cap3.isOpened())
		return -1;

	createTrackbar("THRESH_VALUE", "Calibrate Left", &thresh2, 255, 0, NULL);
	createTrackbar("CUTOFF_VALUE", "Calibrate Left", &cutoff, 10000, 0, NULL);
	createTrackbar("THRESH_VALUE", "Calibrate Right", &thresh, 255, 0, NULL);
	createTrackbar("CUTOFF_VALUE", "Calibrate Right", &cutoff, 10000, 0, NULL);

	//scales with the center of selection as scaling center
	dst_left[0].x = (-1)*x_left+320;    //3*x
	dst_left[0].y = (1)*x_left+240;
	dst_left[1].x = (-1)*x_left+320;    //3*x
	dst_left[1].y = (-1)*x_left+240;
	dst_left[2].x = (1)*x_left+320;
	dst_left[2].y = (-1)*x_left+240;
	dst_left[3].x = (1)*x_left+320;
	dst_left[3].y = (1)*x_left+240;
	cal_dist_left = (df(src_left[0].x, src_left[0].y, src_left[1].x, src_left[1].y) +
		     df(src_left[1].x, src_left[1].y, src_left[2].x, src_left[2].y) +
		     df(src_left[2].x, src_left[2].y, src_left[3].x, src_left[3].y) +
		     df(src_left[3].x, src_left[3].y, src_left[0].x, src_left[0].y))*0.25;
	cal_dist_left = cal_dist / cal_dist_left;	
	trans_left = getPerspectiveTransform(src_left,dst_left);				
	//scales with the center of selection as scaling center
	dst_right[0].x = (-1)*x_right+320;    //3*x
	dst_right[0].y = (1)*x_right+240;
	dst_right[1].x = (-1)*x_right+320;    //3*x
	dst_right[1].y = (-1)*x_right+240;
	dst_right[2].x = (1)*x_right+320;
	dst_right[2].y = (-1)*x_right+240;
	dst_right[3].x = (1)*x_right+320;
	dst_right[3].y = (1)*x_right+240;
	cal_dist_right = (df(src_right[0].x, src_right[0].y, src_right[1].x, src_right[1].y) +
		     df(src_right[1].x, src_right[1].y, src_right[2].x, src_right[2].y) +
		     df(src_right[2].x, src_right[2].y, src_right[3].x, src_right[3].y) +
		     df(src_right[3].x, src_right[3].y, src_right[0].x, src_right[0].y))*0.25;
	cal_dist_right = cal_dist / cal_dist_right;
	trans_right = getPerspectiveTransform(src_right,dst_right);

	while(true)
	{
		cap2 >> rightim;
		cap3 >> leftim;

		split(rightim, planes_right);
		split(leftim, planes_left);

		GaussianBlur(planes_right[0],planes_right[0],Size(0,0),2,2);
		GaussianBlur(planes_right[1],planes_right[1],Size(0,0),2,2);
		GaussianBlur(planes_right[2],planes_right[2],Size(0,0),2,2);
		
		GaussianBlur(planes_left[0],planes_left[0],Size(0,0),2,2);
		GaussianBlur(planes_left[1],planes_left[1],Size(0,0),2,2);
		GaussianBlur(planes_left[2],planes_left[2],Size(0,0),2,2);

		threshold(planes_right[0], planes_right[0], thresh, 255.0, 0);
		threshold(planes_right[1], planes_right[1], thresh, 255.0, 0);
		threshold(planes_right[2], planes_right[2], thresh, 255.0, 0);
		threshold(planes_left[0], planes_left[0], thresh2, 255.0, 0);
		threshold(planes_left[1], planes_left[1], thresh2, 255.0, 0);
		threshold(planes_left[2], planes_left[2], thresh2, 255.0, 0);

		lines_left = planes_left[0] & planes_left[1] & planes_left[2];
		lines_right = planes_right[0] & planes_right[1] & planes_right[2];

		findContours(lines_left, countours_left, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		findContours(lines_right, countours_right, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);	

		it = countours_left.begin();
		while(it != countours_left.end())
		{
			if(abs(contourArea(*it)) < cutoff)
			{
				if(it != countours_left.begin())
				{
				it--;
				it2 = it;				
				it++;
				countours_left.erase(it);
				it = it2;
				}
				else
				{
					while(abs(contourArea(*it)) < cutoff)
					{
						countours_left.erase(it);
						if(countours_left.size() == 0)
						{
							it = countours_left.begin();
							break;
						}
						it = countours_left.begin();
					}
				}
			}
			if(countours_left.size() != 0)
			{
				it++;
			}
		}

		it = countours_right.begin();
		while(it != countours_right.end())
		{
			if(abs(contourArea(*it)) < cutoff)
			{
				if(it != countours_right.begin())
				{
				it--;
				it2 = it;				
				it++;
				countours_right.erase(it);
				it = it2;
				}
				else
				{
					while(abs(contourArea(*it)) < cutoff)
					{
						countours_right.erase(it);
						if(countours_right.size() == 0)
						{
							it = countours_right.begin();
							break;
						}
						it = countours_right.begin();
					}
				}
			}
			if(countours_right.size() != 0)
			{
				it++;
			}
		}

		cont_left = Mat::zeros(lines_left.rows, lines_left.cols, lines_left.type());
		cont_right = Mat::zeros(lines_right.rows, lines_right.cols, lines_right.type());

		drawContours(cont_left, countours_left, -1, CV_RGB(255, 255, 255), 10, 8);
		drawContours(cont_right, countours_right, -1, CV_RGB(255, 255, 255), 10, 8);

		imshow("Left", leftim);
		imshow("Right", rightim);
		imshow("Composite Left" , cont_left);
		imshow("Composite Right" , cont_right);

		warpPerspective(cont_left,frame_trans_left,trans_left,cont_left.size());
		imshow("Calibrate Left",frame_trans_left);
		dist_left = 639;
		for(counter = 0; counter < frame_trans_left.rows; counter++)
		{
			for(counter2 = 0; counter2 < frame_trans_left.cols; counter2++)
			{
				if(frame_trans_left.at<bool>(counter, counter2) == 255 && dist_left > (639 - counter2))
					{
					dist_left = 639 - counter2;
					}
			}
		}
		real_dist_left = cal_dist_left * dist_left;

		warpPerspective(cont_right,frame_trans_right,trans_right,cont_right.size());
		imshow("Calibrate Right",frame_trans_right);
		dist_right = 639;
		for(counter = 0; counter < frame_trans_right.rows; counter++)
		{
			for(counter2 = 0; counter2 < frame_trans_right.cols; counter2++)
			{
				if(frame_trans_right.at<bool>(counter, counter2) == 255 && dist_right > counter2)
					{
					dist_right = counter2;
					}
			}
		}
		real_dist_right = cal_dist_right * dist_right;

		if(waitKey(30) >= 0)
			break;
	}

	return 0;
}

double df(int x1, int y1, int x2, int y2)
{
	
	return sqrt(((double)x2 - (double)x1)*((double)x2 - (double)x1) + ((double)y2-(double)y1)*((double)y2-(double)y1));
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
