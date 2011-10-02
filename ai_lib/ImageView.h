/*
 *  Roomba.h
 *  roomba
 *
 */

#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H


//c++ libs
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

//c libs
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

#include <cvaux.h>
#include <highgui.h>

#define IMAGE_HEIGHT 480
#define IMAGE_WIDTH  640

using namespace std;

class ImageView
{
public:
	ImageView();
	
	void load(IplImage*);

	//const CvMat& Obstacles();
	//const CvMat& Lines();
	
private:
	CvMat* obstacles;
	CvMat* lines;
    IplImage* current_image, hsv_image, histogram_mask, threshold_result, hue_channel, value_channel;
	CvHistogram* obstacle_hue_histogram;
	CvHistogram* obstacle_val_histogram;
	CvHistogram* h_hue, h_val; /* used in each calculation loop */
	CvScalar hsv_min, hsv_max;
	CameraCap* cap; /* capture class with ->height, ->width, ->size, ->left, ->right, and ->getimage() */
	const int hist_hue_size, hist_val_size, hist_hue_max, hist_val_max;
	int hue_cutoff, val_cutoff;
	
	/* line format? binary matrix? line points? */
	/* Koushik */ void detectLines();
	
	/* Update obstacles matrix with 1=obstacle, 0=free space */
	/* Ian */ void detectObstacles();
	
	/* update histogram with data from previous samples declared safe */
	/* Ian */ void updateHistogram(CvHistogram*, CvHistogram*);
	
};

