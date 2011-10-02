#include <iostream>
#include <libplayerc++/playerc++.h>
#include <math.h>

//for opencv c
#include <stdio.h>
#include <cv.h>
#include <highgui.h>

using namespace PlayerCc;
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	VideoCapture cap(1);
	
	if(!cap.isOpened())
		return -1;

	Mat frame;

	namedWindow("result",1);

	while(true)
	{
		cap >> frame;
		
		imshow("result",frame);

		if(waitKey(30) >= 0)
			break;
	}

	return 0;
}


