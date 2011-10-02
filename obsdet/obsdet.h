#include "cv.h"

typedef struct ll{
	int x1;
	int x2;
	int y1;
	int y2;
	struct ll * next;
}llnode;

	
	
int detect_obstacles();
//int single_frame_detect_obstacles();
int process_frame(IplImage* frame, IplImage* h_plane, IplImage* v_plane, CvHistogram* hist_h, CvHistogram* hist_v, int Hthresh, int Vthresh);
CvHistogram* create_histogram(IplImage* plane, int range, int* bins);
