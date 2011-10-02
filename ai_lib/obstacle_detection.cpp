#include "obstacle_detection.h"
#include <math.h>
#include <stdio.h>

using namespace cv;

void ObstacleDetection::clear() {
	cur_mask_h.setTo(0);
	cur_mask_v.setTo(0);
	objects.clear();
}
void ObstacleDetection::load_mask(const char* maskfile) {
	hist_mask = imread(maskfile, 0);
}
void ObstacleDetection::load_hist(const char* histfile) {
	exit(-1);
}

ObstacleDetection::ObstacleDetection() {
	clear();
}

ObstacleDetection::ObstacleDetection(const char* maskfile) {
	clear();
	// Load mask
	load_mask(maskfile);
}

ObstacleDetection::ObstacleDetection(const char* maskfile, const char* histfile) {
	clear();
	// Load mask
	load_mask(maskfile);
	load_hist(histfile);
}

void ObstacleDetection::operator()(Mat target) {
	clear();
	// generate histogram
	// hue
	cur_frame = target;
	int h_channel[] = {H_CHANNEL};
	int v_channel[] = {V_CHANNEL};
	float h_ranges[] = { 0, (float)H_MAX };
	float v_ranges[] = { 0, (float)V_MAX };
	const float* h_range[] = {h_ranges};
	const float* v_range[] = {v_ranges};
	int histSize[] = {H_BINS};
    int histSize1[] = {V_BINS};

	//    const float* ranges[] = { hranges, sranges };

	calcHist(&target, 1, h_channel, Mat(), h_hue, 1, histSize,h_range,true,false);
	calcHist(&target, 1, v_channel, Mat(), h_val, 1, histSize1, v_range,true,false);

	classify_image();

	// group results
	make_groups();
}

vector<Rect>* ObstacleDetection::get_objects() {
	return &objects;
}

Mat ObstacleDetection::get_obstacle_mask() {
	return cur_mask;
}

void ObstacleDetection::save_to_file(const char* filename) {
	/* ... */
}

void ObstacleDetection::classify_image() {
	int bin;
	Vec3b pixel;
	// histogram loops
	for (int x=0; x<cur_frame.cols; x++) {
		for (int y=0; y<cur_frame.rows; y++) {
			// hue first
			pixel = cur_frame.at<Vec3b>(x,y);
			printf("%d,%d,%d", pixel[0], pixel[1], pixel[2]);
		  	bin = floor(pixel[H_CHANNEL] / H_MAX);
			if (h_hue.at<int>(bin) < H_THRESH_VAL) {
				cur_mask_h.at<bool>(x,y) = bin; // obstacle
				continue;
			} else {
				cur_mask_h.at<bool>(x,y) = 0;
			}

			// val
			bin = pixel[V_CHANNEL] / V_MAX;
			if (h_val.at<int>(0,bin) < V_THRESH_VAL) {
				cur_mask_v.at<bool>(x,y) = bin;
			} else {
				cur_mask_v.at<bool>(x,y) = 0;
			}
		}
	}
	bitwise_or(cur_mask_v, cur_mask_h,cur_mask);
}

void ObstacleDetection::make_groups() {

	// group by bin values
	// start in one corner, create a temporary blob object, and keep filling out
	// to search for connected obstacle pixels with the same or similar values
	// if a pixel doesn't match but is an obstacle, add it to a queue to be processed
	objects.clear();
	/* ... */
}

void ObstacleDetection::average_hist(Mat orig, Mat next) {
	assert(orig.rows == next.rows);
	for (int i=0; i < orig.rows; i++) {
	  orig.at<int>(0,i) = (int) (orig.at<int>(0,i)*(1-next_weight) + next.at<int>(0,i)*next_weight);
	}
}
