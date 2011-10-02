/*
 * Stereo code
 */

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>

using namespace cv;

// the y disparity allowed (in pixels)
// increasing this might introduce errors+run slower
#define SCANLINE_DELTA 10

// camera parameters
#define BASELINE 13.0 // in CM between cameras
#define FOCAL_LENGTH 100 // by testing

/* needs to implement CV_TM_SQDIFF along with a mask (only take the difference
 * if mask at that point == 1
 *
 * Result(x,y) = Sum over x',y' of pixels in Template of (Mask(x',y')) : (Template(x',y') - Source(x+x', y+y'))^2 ? 0;
 */

void maskedMatchTemplate(Mat& templ, Mat& search, Mat& result, Mat& mask) {
	/* check result dimensions */

	result.create(abs(search.rows - templ.rows) + 1, abs(search.cols - templ.cols) + 1, CV_64F);
	//printf("Dims: %dx%d, %dx%d, %dx%d, %dx%d", templ.rows, templ.cols, search.rows, search.cols, result.rows, result.cols, mask.rows, mask.cols);
	result.setTo(0);
	for (int i=0; i < result.rows; i++) {
		for (int j=0; j < result.cols; j++) {
			// sum for this template at offset i,j
			for (int it=0; it < templ.rows; it++) {
				for (int jt=0; jt < templ.cols; jt++) {
					if (mask.at<int>(it,jt) == 1)
					{
						result.at<double>(i,j) += pow(templ.at<int>(it,jt) - search.at<int>(i+it,j+jt), 2);
						//printf("%f", result.at<double>(i,j));
					}
				}
			}
		}
	}

}

Point3f stereo_calc(Point left, Point right) {
	Point3f loc;
	loc.z = (BASELINE * FOCAL_LENGTH) / (left.x - right.x);
	loc.x = (left.x * loc.z) / FOCAL_LENGTH;
	loc.y = (left.y * loc.z) / FOCAL_LENGTH;
	return loc;
}

// mask and coords are in the left image, searching right image
void find_depths(vector<Rect>* coords, Mat left, Mat right, Mat mask, vector<Point3f>* return_points) {
	Point3f location;
	Point min_point;
	// make local copies of matrix data
	Mat copy_left;
	Mat copy_right;
	right.copyTo(copy_right);
	left.copyTo(copy_left);
	Mat bw_right, bw_left;
	Mat match_region;
	Mat obstacle;
	Mat obstacle_mask;
	Mat result;
	double test=0.0;

	// convert to gray, mask background

	cvtColor(copy_right, bw_right, CV_RGB2GRAY);
	cvtColor(copy_left, bw_left, CV_RGB2GRAY);
	//bitwise_and(bw_left, mask, bw_left);

	for(vector<Rect>::const_iterator it = coords->begin(); it != coords->end(); ++it) {
		// allocate result matrix
		// result matrix should be (W - w + 1) x (H - h + 1) where WxH are right subimage dimensions, wxh are object dimensions
		// Right image obstacle ALWAYS to the left of the left image obstacle, +- SCANLINE_DELTA pixels in the y direction
		result.create(Size((*it).x + 1, SCANLINE_DELTA*2 + 1), CV_8UC(15));

		// cut obstacle
		obstacle = bw_left(*it); // set the image ROI to the current Rect of interest (left eye)
		obstacle_mask = mask(*it);
		//imshow("obstacle", obstacle);
		//imshow("obstacle_mask", obstacle_mask);
		// set match region (x,y,width,height)
		match_region = bw_right(Rect(0, (*it).y-SCANLINE_DELTA, (*it).x + (*it).width, (*it).height + 2*SCANLINE_DELTA));
		//imshow("match_region", match_region);

		// match
		maskedMatchTemplate(match_region, obstacle, result, obstacle_mask);
		//imshow("result", result);

		// find best result (minimum)
		minMaxLoc(result, &test, NULL, &min_point, NULL);
		// calculate range
		//printf("(%d,%d)=%f, (%d,%d), %d\n", test, min_point.x, min_point.y, (*it).x, (*it).y, (*it).x-min_point.x);
		location = stereo_calc(Point((*it).x, (*it).y), min_point);

		// store
		return_points->push_back(location);
	}
}


