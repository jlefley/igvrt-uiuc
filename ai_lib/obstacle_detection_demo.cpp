#include "obstacle_detection.h"
#include <stdio.h>

#define DEBUG 1

/*
 * ./obstacle_detection_demo mask.png
 */
int main(int argc, char** argv) {
	ObstacleDetection detector(argv[1]);
	Mat eye_left, eye_left_hsv, eye_right;
	VideoCapture left(0);
	if (DEBUG) {
		cvNamedWindow("Capture", 1);
	}
	vector<Point3f> depths;
	while(1) {
	  	left >> eye_left;
	  	eye_left_hsv.create(eye_left.rows, eye_left.cols, sizeof(char));
	  	cvtColor(eye_left, eye_left_hsv, CV_BGR2HSV);
	  	// convert to HSV
	  	
	  	//if (eye_left == NULL) {continue;}
	  	if (DEBUG) {
	  		imshow("Capture", eye_left);
	  	}
		detector(eye_left);
		//	find_depths(detector.get_objects(), eye_left, eye_right, detector.get_obstacle_mask(), &depths);
		waitKey(10);
	}
}
