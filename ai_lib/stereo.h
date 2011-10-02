/*
 * stereo.h
 *
 */
#include <vector>
#include <opencv/cv.h>
using namespace cv;

#ifndef STEREO_H_
#define STEREO_H_

void maskedMatchTemplate(Mat& templ, Mat& search, Mat& result, Mat& mask);
Point3f stereo_calc(Point left, Point right);
void find_depths(vector<Rect>* coords, Mat left, Mat right, Mat mask, vector<Point3f>* result);

#endif /* STEREO_H_ */
