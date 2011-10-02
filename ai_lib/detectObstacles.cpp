#include <ImageView.h>

void ImageView::detectObstacles() {
    CvHistogram* h_hue;
    CvHistogram* h_val;
	int i,j;
	int hue, val, hue_bin, val_bin;
    cvCvtColor(hsv_image, current_image, CV_BGR2HSV);
    cvSmooth(hsv_image, hsv_image, CV_GAUSSIAN, 5, 5);
    cvInRangeS(hsv_image, hsv_min, hsv_max, thresh_mask);
	cvAnd(thresh_mask, histogram_mask, thresh_mask);

	/* Hue[0,180], Sat[0,255], Val[0,255] */
	cvSplit(hsv_image, hue_channel, NULL, NULL, NULL);
	cvSplit(hsv_image, NULL, NULL, value_channel, NULL);

	cvCalcHist(hue_channel, h_hue, 0, histogram_mask);
	cvCalcHist(value_channel, h_val, 0, histogram_mask);

	updateHistogram(h_hue, h_val);
	
	cvZero(obstacles);
	for (i=0; i<cap->width; i++) {
		for (j=0; j<cap->height; j++) {
			hue = cvGetReal2D(hue_channel, j, i);
			hue_bin = cvCeil((hue * hist_hue_size) / (double) hist_hue_max);
			if (hue_bin < 0) {hue_bin = 0;}
			if (cvGetReal1D(obstacle_hue_histogram->bins, hue_bin) < hue_cutoff) {
				cvSetReal2D(obstacles, j, i, 1);
				continue;
			}
			val = cvGetReal2D(value_channel, j, i);
			val_bin = cvCeil((val * hist_val_size) / (double) hist_val_max);
			if (val_bin < 0) {val_bin = 0;}
			if (cvGetReal1D(obstacle_val_histogram->bins, val_bin) < val_cutoff) {
				cvSetReal2D(obstacles, j, i, 1);
				continue;
			}
		}
	}
}

void ImageView::updateHistogram(CvHistogram* hue, CvHistogram* val) {
    int i;
	for (i=0; i<hist_hue_size; i++) {
		cvSetReal1D(obstacle_hue_histogram->bins, i, cvMax(cvGetReal1D(hue->bins, i), cvGetReal1D(obstacle_hue_histogram->bins, i)));
	}
	for (i=0; i<hist_val_size; i++) {
		cvSetReal1D(obstacle_val_histogram->bins, i, cvMax(cvGetReal1D(val->bins, i), cvGetReal1D(obstacle_val_histogram->bins, i)));
	}
}
