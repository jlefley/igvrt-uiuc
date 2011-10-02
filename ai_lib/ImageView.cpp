#include <ImageView.h>
#define MASK_FILE "./obstacle_mask.png"

ImageView::ImageView() {
    hist_hue_size = 20;
    hist_val_size = 32;
	hist_hue_max = 180;
	hist_val_max = 256;
    const float* range_hue[] = {{0,hist_hue_max}};
    const float* range_val[] = {{0,hist_val_max}};
    int sample_pix = 0;
    const int total_pix = IMAGE_HEIGHT*IMAGE_WIDTH;
    int i;
    int j;
    hsv_min = cvScalar(0, 30, 10, 0);
    hsv_max = cvScalar(180, 256, 256, 0);
    float hue_cutoff, val_cutoff;
    histogram_mask = cvLoadImage(MASK_FILE);
    for (i=0;i<IMAGE_WIDTH;i++) {
        for (j=0;j<IMAGE_HEIGHT;j++) {
            if (cvGetReal2D(mask_image, j, i) > 0) {
                sample_pix++;
            }
        }
    }
	
    hue_cutoff = 0.01*sample_pix;
    val_cutoff = 0.015*sample_pix;
    
    obstacle_hue_histogram = cvCreateHist(1, &hist_hue_size, CV_HIST_ARRAY, range_hue, 1);
    obstacle_val_histogram = cvCreateHist(1, &hist_val_size, CV_HIST_ARRAY, range_val, 1);
    h_hue = cvCreateHist(1, &hist_hue_size, CV_HIST_ARRAY, range_hue, 1);
    h_val = cvCreateHist(1, &hist_val_size, CV_HIST_ARRAY, range_val, 1);
	
    obstacles = cvCreateMat(MAP_HEIGHT, MAP_WIDTH, CV_1UC1);
    lines = cvCreateMat(MAP_HEIGHT, MAP_WIDTH, CV_1UC1);
    hsv_image = cvCreateImage(cap->size, 8, 1);
	threshold_result = cvCreateImage(cap->size, 8, 1);
	hue_channel = cvCreateImage(cap->size, 8, 1);
	value_channel = cvCreateImage(cap->size, 8, 1);
	
}
