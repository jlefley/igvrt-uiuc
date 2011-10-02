/*
 * Line detection
 */
 
#include "lines.h"
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <vector>

using namespace cv;

/*
 * Takes a 3-channel RGB image and a 1-channel 8-bit BW output image
 */

void detect_lines(Mat& source_rgb, Mat& output_bw) {
	// convert to gray
	Mat calculation(source_rgb.size(), CV_8UC1);
	Mat binary(source_rgb.size(), CV_8UC1);
	
	vector<Vec4i> lines;
	cvtColor(source_rgb, calculation, CV_BGR2GRAY);
	threshold(calculation, binary, 220.0, 225.0, CV_THRESH_BINARY);
	Canny(binary, calculation, 120, 120*3, 3);
	HoughLinesP(calculation, lines, 1, CV_PI/180, 80, 30, 10);
	// draw lines
	output_bw.create(source_rgb.size(), CV_8UC1);
	output_bw.setTo(0);
	for (int i=0; i<lines.size(); i++) {
		line(output_bw, 
			Point(lines[i][0], lines[i][1]), 
			Point(lines[i][2], lines[i][3]),
			Scalar(180),
			3, 8
		);
	}
}
