#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#define H_BINS 12
#define V_BINS 18
#define H_MAX 360.0
#define V_MAX 100.0
#define H_THRESH_VAL 0.001
#define V_THRESH_VAL 0.03
#define H_CHANNEL 0
#define V_CHANNEL 2
#define next_weight 0.05
using namespace cv;

class ObstacleDetection {
public:
	ObstacleDetection();
	ObstacleDetection(const char* maskfile);
	ObstacleDetection(const char* maskfile, const char* histfile);
	/*
	 * Main calculation function
	 * 	   Clear all memory
	 *     Calculate histograms for current frame
	 *     Classify image pixels with existing histogram
	 *     If no obstacles in mask region:
	 *         Average current and original histograms
	 *     Calculate connected regions
	 *     Return bounding boxes
	 */
	void operator()(Mat target);
	void save_to_file(const char* filename);
	vector<Rect>* get_objects();
	Mat get_obstacle_mask();
private:
	/*
	 * Resets Mats to 0
	 */
	void clear();
	MatND h_hue, h_val; // hue and value histograms
	Mat hist_mask; // mask to use when calculating histogram (bottom center portion)
	Mat cur_frame; // holds current raw frame to be processed
	Mat cur_mask_h; // result of image classified with hue histogram
	Mat cur_mask_v;
	Mat cur_mask; // = cur_mask_h OR cur_mask_v
	vector<Rect> objects; // list of identified objects that need depths
	
	void load_mask(const char* maskfile);
	void load_hist(const char* histfile);
	/*
	 * Updates orig as the weighted average of bins in orig and next
	 * 0.95 and 0.05 orig to next weighting
	 */
	void average_hist(Mat orig, Mat next);

	/*
	 * Fills in cur_mask_v and cur_mask_h with bin values based upon h_val and h_hue
	 */
	void classify_image();

	/*
	 * Returns bounding boxes around connected regions in cur_mask
	 */
	void make_groups();
};
