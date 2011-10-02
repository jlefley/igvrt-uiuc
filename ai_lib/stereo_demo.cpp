/*
 * stereo_demo.cpp
 *
 */

#include "stereo.h"
#include <stdlib.h>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;
struct demostate {
	Rect curtarget;
	int current_state;
	vector<Rect> defined_targets;
};

typedef struct demostate DemoState;

void window_callback(int event, int x, int y, int flags, void* state) {
	DemoState* cast_state = (DemoState*) state;
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			// set x,y
			cast_state->curtarget.x = x;
			cast_state->curtarget.y = y;
			cast_state->curtarget.width = 1;
			cast_state->curtarget.height = 1;
			cast_state->current_state = CV_EVENT_LBUTTONDOWN;
			break;
		case CV_EVENT_LBUTTONUP:
			// set final width
			if (cast_state->current_state == CV_EVENT_LBUTTONDOWN) {
				cast_state->curtarget.width = abs(cast_state->curtarget.x-x);
				cast_state->curtarget.height = abs(cast_state->curtarget.y-y);
				// add to vector
				cast_state->defined_targets.push_back(cast_state->curtarget);
				cast_state->current_state = -1;
			}

			break;
		case CV_EVENT_RBUTTONDOWN:
			if (!cast_state->defined_targets.empty()) {
				cast_state->defined_targets.pop_back();
			}
			break;
	}
}
// $./stereo_test left.png right.png mask.png


int main(int argc, char* argv[]) {
	Mat left;
	Mat right;
	Mat mask;
	Mat demoimg;
	DemoState state;
	char buffer[100];
	vector<Point3f> results;
	vector<Point3f>::const_iterator pointit;
	int i;
	// load images as grayscale
	left = imread(argv[1]);
	right = imread(argv[2]);
	mask = imread(argv[3], 0);
	namedWindow("Stereo Demo", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("Stereo Demo", window_callback, &state);

	while (i != 'q') {
		left.copyTo(demoimg); // copy for this frame
		// call stereo code
		results.erase(results.begin(), results.end());
		find_depths(&state.defined_targets, left, right, mask, &results);

		// draw results
		pointit = results.begin();
		for (vector<Rect>::const_iterator it = state.defined_targets.begin(); it != state.defined_targets.end(); ++it) {
			rectangle(demoimg, Point(it->x, it->y), Point(it->x+it->width, it->y+it->height), Scalar(255,0,0));
			sprintf(buffer, "(%.2f, %.2f, %.2f)",pointit->x, pointit->y, pointit->z);
			putText(demoimg, buffer, Point(it->x, it->y), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.6, Scalar(255,255,0));
			++pointit;
		}
		imshow("Stereo Demo", demoimg);
		i = waitKey(30);
	}

	return 0;
}
