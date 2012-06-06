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

int thresh = 125; 
int thresh2 = 125;

int cutoff = 1000;


Mat leftim, rightim, centerim, trans_left, trans_right, frame_trans_left, frame_trans_right;
vector<Mat> planes_left;
vector<Mat> planes_right;
Mat lines_left, left1;
Mat lines_right, right1;
Mat cont_left;
Mat cont_right;
vector<vector<Point> > countours_left, countours_right;
vector<vector<Point> >::iterator it, it2;
vector<Vec4f> fitLines_left, fitLines_right;
vector<CvPoint> sort_left, sort_right;
vector<CvPoint>::iterator it3;
Vec4f line1;
CvPoint pt1,pt2;
vector<Point> a, b;

Point2f src_left[4], src_right[4], dst_left[4], dst_right[4];
int pt_ind_left = 0;
int pt_ind_right = 0;

int x_left = 100;
int x_right = 100;

void my_mouse_callback_left(int event, int x, int y, int flags, void* param);
void my_mouse_callback_right(int event, int x, int y, int flags, void* param);

int main(int argc, char *argv[])
{
	namedWindow("Left",1);
	namedWindow("Right",1);
	namedWindow("Composite Left",1);
	namedWindow("Composite Right",1);
	namedWindow("Lines Left",1);
	namedWindow("Lines Right",1);
	namedWindow("Calibrate Left", 1);
	namedWindow("Calibrate Right", 1);
	namedWindow("Cont_Left", 1);
	namedWindow("Cont_Right", 1);

	int counter = 0;
	int counter2 = 0;

	int dist_left = 0;
	int dist_right = 0;

	VideoCapture cap1(1); //1
	VideoCapture cap2(2); //2
	VideoCapture cap3(3); //3

	if(!cap1.isOpened() || !cap2.isOpened() || !cap3.isOpened())
		return -1;

	createTrackbar("t0", "Calibrate Left", &x_left, 1000, 0, NULL);
	cvSetMouseCallback("Calibrate Left", my_mouse_callback_left, NULL);

	createTrackbar("t1", "Calibrate Right", &x_right, 1000, 0, NULL);
	cvSetMouseCallback("Calibrate Right", my_mouse_callback_right, NULL);

	createTrackbar("t_l", "Composite Left", &thresh2, 255, 0, NULL);
	createTrackbar("cutoff", "Composite Left", &cutoff, 10000, 0, NULL);
	createTrackbar("t_r", "Composite Right", &thresh, 255, 0, NULL);
	createTrackbar("cutoff", "Composite Right", &cutoff, 10000, 0, NULL);

	while(true)
	{
		cap1 >> centerim;
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

		//left1 = lines_left.clone();
		//right1 = lines_right.clone();		

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

		drawContours(cont_left, countours_left, -1, CV_RGB(56, 17, 125), 10, 8);
		drawContours(cont_right, countours_right, -1, CV_RGB(56, 17, 125), 10, 8);
		/*
		sort_left.clear();
		sort_right.clear();

		for(counter = 0; counter < countours_left.size(); counter++)
		{
			line1[0] = 0.0;
			line1[1] = 0.0;
			line1[2] = 0.0;
			line1[3] = 0.0;
			fitLines_left.push_back(line1);
			fitLine(countours_left[counter], fitLines_left[counter], CV_DIST_L2, 0, 0.01, 0.01);
			line1 = fitLines_left[counter];
			pt1.x = line1[2];
			pt1.y = line1[3];
			
			it3 = sort_left.begin();
			while(it3 != sort_left.end())
			{
				if((*it3).y > pt1.y)
				{
					break;
				}
				it3++;
			}
			sort_left.insert(it3, pt1);
			}

		for(counter = 0; counter < countours_right.size(); counter++)
		{
			line1[0] = 0.0;
			line1[1] = 0.0;
			line1[2] = 0.0;
			line1[3] = 0.0;
			fitLines_right.push_back(line1);
			fitLine(countours_right[counter], fitLines_right[counter], CV_DIST_L2, 0, 0.01, 0.01);
			line1 = fitLines_right[counter];
			pt1.x = line1[2];
			pt1.y = line1[3];
			
			it3 = sort_right.begin();
			while(it3 != sort_right.end())
			{
				if((*it3).y > pt1.y)
				{
					break;
				}
				it3++;
			}
			sort_right.insert(it3, pt1);
		}

		for(counter = 0; counter < sort_left.size(); counter++)
		{
			cout << "Left " << counter << ": ("<< sort_left[counter].x << " ," << sort_left[counter].y << ")" << endl;
			if(counter == sort_left.size() - 1)
			{
				break;
			}
			line(left1, sort_left[counter], sort_left[counter + 1], CV_RGB(124, 14, 65), 5, CV_AA, 0);
		}

		for(counter = 0; counter < sort_right.size(); counter++)
		{			
			cout << "Right " << counter << ": ("<< sort_right[counter].x << " ," << sort_right[counter].y << ")" << endl;
			if(counter == sort_right.size() - 1)
			{
				break;
			}
			line(right1, sort_right[counter], sort_right[counter + 1], CV_RGB(124, 14, 65), 5, CV_AA, 0);
		}*/

		imshow("Left", leftim);
		imshow("Right", rightim);
		imshow("Composite Left" , lines_left);
		imshow("Composite Right" , lines_right);
		imshow("Cont_Left", cont_left);
		imshow("Cont_Right", cont_left);

		

		/*cout << sort_left.size() << " " << sort_right.size() << endl;
		imshow("Lines Left" , left1);
		imshow("Lines Right" , right1);*/

		if(pt_ind_left >= 4)
		{
			//scales with the center of selection as scaling center
			dst_left[0].x = (-1)*x_left+320;    //3*x
			dst_left[0].y = (1)*x_left+240;

			dst_left[1].x = (-1)*x_left+320;    //3*x
			dst_left[1].y = (-1)*x_left+240;
	
			dst_left[2].x = (1)*x_left+320;
			dst_left[2].y = (-1)*x_left+240;

			dst_left[3].x = (1)*x_left+320;
			dst_left[3].y = (1)*x_left+240;


			trans_left = getPerspectiveTransform(src_left,dst_left);
			warpPerspective(cont_left,frame_trans_left,trans_left,cont_left.size());
			imshow("Calibrate Left",frame_trans_left);
			for(counter = 0; counter < frame_trans_left.rows(); counter++)
			{
				for(counter2 = 0; counter2 < frame_trans_left.cols(); counter2++)
				{
					if(frame_trans_left.at<int>(counter2, counter) == 255 && dist_left < counter2)
						{
						dist_left = counter2;
						}
				}
			}
			cout << "dist_left: " << dist_left << endl;
		}
		else
		{
			imshow("Calibrate Left",leftim);
		}

		
		if(pt_ind_right >= 4)
		{
			//scales with the center of selection as scaling center
			dst_right[0].x = (-1)*x_right+320;    //3*x
			dst_right[0].y = (1)*x_right+240;

			dst_right[1].x = (-1)*x_right+320;    //3*x
			dst_right[1].y = (-1)*x_right+240;
	
			dst_right[2].x = (1)*x_right+320;
			dst_right[2].y = (-1)*x_right+240;

			dst_right[3].x = (1)*x_right+320;
			dst_right[3].y = (1)*x_right+240;


			trans_right = getPerspectiveTransform(src_right,dst_right);
			warpPerspective(cont_right,frame_trans_right,trans_right,cont_right.size());
			imshow("Calibrate Right",frame_trans_right);
			for(counter = 0; counter < frame_trans_right.rows(); counter++)
			{
				for(counter2 = 0; counter2 < frame_trans_right.cols(); counter2++)
				{
					if(frame_trans_right.at<int>(counter2, counter) == 255 && dist_right < counter2)
						{
						dist_right = counter2;
						}
				}
			}
			cout << "dist_right: " << dist_right << endl;
		}
		else
		{
			imshow("Calibrate Right",rightim);
		}

		if(waitKey(30) >= 0)
			break;
	}

	return 0;
}

void my_mouse_callback_left(int event, int x, int y, int flags, void* param)
{
	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:
	
			if(pt_ind_left < 4)
			{
				cout << x << " " << y << endl;
				src_left[pt_ind_left].x = x;
				src_left[pt_ind_left].y = y;
				pt_ind_left++;
			}
			break;
	}
}

void my_mouse_callback_right(int event, int x, int y, int flags, void* param)
{
	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:
	
			if(pt_ind_right < 4)
			{
				cout << x << " " << y << endl;
				src_right[pt_ind_right].x = x;
				src_right[pt_ind_right].y = y;
				pt_ind_right++;
			}
			break;
	}
}

