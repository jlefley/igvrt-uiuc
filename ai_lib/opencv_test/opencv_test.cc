#include <iostream>
#include <libplayerc++/playerc++.h>
#include <math.h>

//for opencv c
#include <stdio.h>
#include <cv.h>
#include <highgui.h>

using namespace PlayerCc;
using namespace std;
using namespace cv;

int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat src, src_gray, dst, src1, hsv;
MatND hist;
char* window_name = "Threshold Demo";

char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";
/*
function populate_histogram
*/


/**
 * @function Threshold_Demo
 */
void Threshold_Demo( int, void* )
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */

  threshold( src_gray, dst, threshold_value, max_BINARY_value,threshold_type );

  imshow( window_name, dst );
}


int main(int argc, char *argv[])
{
	int histogram[256];
	int counter = 0;
	int rows = 0;
	int columns = 0;

	//FOR HISTOGRAM
        // Quantize the hue to 30 levels
        // and the saturation to 32 levels
        int hbins = 30, sbins = 32;
        int histSize[] = {hbins, sbins};
        // hue varies from 0 to 179, see cvtColor
        float hranges[] = { 0, 180 };
        // saturation varies from 0 (black-gray-white) to
        // 255 (pure spectrum color)
        float sranges[] = { 0, 256 };
        const float* ranges[] = { hranges, sranges };
	// we compute the histogram from the 0-th and 1-st channels
        int channels[] = {0, 1};
	



	VideoCapture cap(1);
	
	if(!cap.isOpened())
		return -1;



	/// Create a window to display results
	namedWindow( window_name, CV_WINDOW_AUTOSIZE );

	/// Create Trackbar to choose type of Threshold
	createTrackbar( trackbar_type, window_name, &threshold_type, max_type, Threshold_Demo );

	createTrackbar( trackbar_value, window_name, &threshold_value, max_value, Threshold_Demo );

	Mat edges;
	Mat edges1;
	Mat edges2;

	namedWindow("original",1);
	namedWindow( "H-S Histogram", 1 );
	namedWindow("canny0",1);
	namedWindow("canny1",1);
	namedWindow("canny2",1);

	while(true)
	{
		//Mat frame;
		//cap >> frame;
		cap >> src;
		cap >> src1;
		imshow("original",src);

		//thresholding
		cvtColor(src,src_gray,CV_BGR2GRAY);
		
		//histogram
	        cvtColor(src1, hsv, CV_BGR2HSV);
		calcHist( &hsv, 1, channels, Mat(), // do not use mask
                hist, 2, histSize, ranges, true, // the histogram is uniform
                false );
		double maxVal=0;
		minMaxLoc(hist, 0, &maxVal, 0, 0);

		int scale = 10;
		Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

		for( int h = 0; h < hbins; h++ )
		for( int s = 0; s < sbins; s++ )
		{
		    float binVal = hist.at<float>(h, s);
		    int intensity = cvRound(binVal*255/maxVal);
		    rectangle( histImg, Point(h*scale, s*scale),
			        Point( (h+1)*scale - 1, (s+1)*scale - 1),
			        Scalar::all(intensity),
			        CV_FILLED );
		}

    
    		imshow( "H-S Histogram", histImg );
		//imshow("result",frame);
		//cvtColor(frame,edges,CV_BGR2GRAY);
		//GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
		//edges1 = edges.clone();
		//edges2 = edges.clone();
	        //Canny(edges, edges, 0, 30, 3);
		//Canny(edges1, edges1, 25, 30, 3);
		//Canny(edges2, edges2, 0, 100, 3);
		//imshow("canny0",edges);
		//imshow("canny1",edges1);
		//imshow("canny2",edges2);
		if(waitKey(30) >= 0)
			break;
	}

	return 0;
}
