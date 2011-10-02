#include <iostream>
#include <libplayerc++/playerc++.h>
#include <deque>
#include <math.h>

using namespace PlayerCc;
using namespace std;

#define LONG_TURN_DELAY 1000000
#define SHORT_TURN_DELAY 300000
#define AFTER_TURN_DELAY 1000000
#define MAX_HITS_BEFORE_LONG_TURN 3
#define TURN_SPEED 1
#define DRIVE_SPEED 0.2
#define NUM_STD_DEVS 3
#define NUM_RUNNING_SAMPLES 7
#define NUM_INITIAL_SAMPLES 100

float updateSensor(deque<float> & sensor, float val, unsigned int maxsamples);
float standardDev(deque<float> & data, float avg);

int main(int argc, char *argv[])
{
	PlayerClient robot("localhost");
	Position2dProxy pp(&robot,0);
	SonarProxy sp(&robot,0);
	IrProxy ip(&robot,0);
	BumperProxy bp(&robot, 0);

	deque<float> sonar;
	deque<float> ir;
	float sonar_cur, ir_cur, sonar_avg, ir_avg, sonar_std, ir_std, sonar_min;
	int numHits = 0;


	for(int i = 0; i < NUM_INITIAL_SAMPLES; i++)
	{
		robot.Read();

		sonar_avg = updateSensor(sonar, sp[0], NUM_INITIAL_SAMPLES);
		ir_avg = updateSensor(ir, ip[0], NUM_INITIAL_SAMPLES);
	}

	sonar_std = standardDev(sonar, sonar_avg);
	ir_std = standardDev(ir, ir_avg);

	while(1)
	{
		robot.Read();

		sonar_cur = updateSensor(sonar, sp[0], NUM_RUNNING_SAMPLES);
		ir_cur = updateSensor(ir, ip[0], NUM_RUNNING_SAMPLES);

		pp.SetSpeed(DRIVE_SPEED, 0);

		sonar_min = (sonar_avg - (NUM_STD_DEVS*sonar_std));
		if(sonar_min < 0)
			sonar_min = 0;

		if(bp[0] || bp [1])
		{
			cout << bp[0] << " " << bp[1] << endl;

			pp.SetSpeed(0, TURN_SPEED);
			usleep(LONG_TURN_DELAY);
			pp.SetSpeed(0, 0);
			usleep(AFTER_TURN_DELAY);
		}
		else if(sonar_cur < sonar_min && ir_cur > (ir_avg + (NUM_STD_DEVS*ir_std)))
		{
			numHits++;

			pp.SetSpeed(0, TURN_SPEED);
			if(numHits > 3)
			{
				usleep(LONG_TURN_DELAY);
				numHits = 0;
			}
			else
			{
				usleep(SHORT_TURN_DELAY);
			}
			pp.SetSpeed(0, 0);
			usleep(AFTER_TURN_DELAY);

			cout << sonar_avg << " " << sonar_std << " " << sonar_cur << endl;
			cout << ir_avg << " " << ir_std << " " << ir_cur << endl << endl;

			sonar.clear();
			ir.clear();
		}
	}
}

float updateSensor(deque<float> & sensor, float val, unsigned int maxsamples)
{
	float avg = 0;

	if(sensor.size() > maxsamples)
		sensor.clear();

	sensor.push_back(val);

	if(sensor.size() > maxsamples)
		sensor.pop_front();

	for(unsigned int i = 0; i < sensor.size(); i++)
	{
		avg += sensor[i];
	}

	avg /= sensor.size();

	return avg;
}

float standardDev(deque<float> & data, float avg)
{
	float std = 0;

	for(unsigned int i = 0; i < data.size(); i++)
	{
		std += pow(data[i] - avg, 2);
	}

	std /= data.size();

	std = pow(std, (float)0.5);

	return std;
}


