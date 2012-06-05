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


Point2f src[4];
Point2f dst[4];
int pt_ind = 0;

void my_mouse_callback(int event, int x, int y, int flags, void* param);

int main(int argc, char *argv[])
{
	VideoCapture cap(-1);
	
	if(!cap.isOpened())
		return -1;

	Mat frame;
	Mat trans;
	Mat frame_trans;

	namedWindow("result",1);

	cvSetMouseCallback("result", my_mouse_callback, NULL);

	int x = 100;
        int b = 5;
	int a = 5;
	
	//create trackbars
	createTrackbar("t0", "result", &x, 1000, 0, NULL);
        createTrackbar("t1", "result", &a, 10, 0, NULL);
	createTrackbar("t2", "result", &b, 10, 0, NULL);

	while(true)
	{
		cap >> frame;

		if(pt_ind >= 4)
		{
//scales with the center of selection as scaling center
			dst[0].x = (-1)*x+320;    //3*x
			dst[0].y = (1)*x+240;

			dst[1].x = (-1)*x+320;    //3*x
			dst[1].y = (-1)*x+240;
	
			dst[2].x = (1)*x+320;
			dst[2].y = (-1)*x+240;

			dst[3].x = (1)*x+320;
			dst[3].y = (1)*x+240;


			trans = getPerspectiveTransform(src,dst);
			warpPerspective(frame,frame_trans,trans,frame.size());
			imshow("result",frame_trans);
		}
		else
		{
			imshow("result",frame);
		}

		if(waitKey(30) >= 0)
			break;
	}

	return 0;
}

void my_mouse_callback(int event, int x, int y, int flags, void* param)
{
	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:
	
			if(pt_ind < 4)
			{
				cout << x << " " << y << endl;
				src[pt_ind].x = x;
				src[pt_ind].y = y;
				pt_ind++;
			}
			break;
	}
}

