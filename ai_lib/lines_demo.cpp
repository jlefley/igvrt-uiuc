#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "lines.h"
#include <stdio.h>

#define DEBUG 1

/*
 * ./lines_demo
 */
int main(int argc, char** argv) {
	Mat camera, depth, lines;
	VideoCapture left(0);
	if (DEBUG) {
		cvNamedWindow("Capture", 1);
		cvNamedWindow("Lines", 1);
	}

	while(1) {
	  	left >> camera;
		
		detect_lines(camera, lines);
		// transform to map view
		
	  	if (DEBUG) {
	  		imshow("Capture", camera);
	  		imshow("Lines", lines);
	  	}
	  	
		waitKey(10);
	}
}
