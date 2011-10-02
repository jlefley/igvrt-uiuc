#include "cv.h"
#include "highgui.h"
#include <ctype.h>
#include <stdio.h>
#include "obsdet.h"

#define H_BINS 10
#define V_BINS 15
#define H_MAX 180
#define V_MAX 256
#define HIST_SELECT 0
#define H_THRESHOLD_VALUE .001
#define V_THRESHOLD_VALUE .03

//using namespace cv;

int detect_obstacles()
{
	CvCapture* capture = 0;
	capture = cvCaptureFromCAM(0);
	IplImage *frame, *imHSV;
    cvNamedWindow("result", 0);	
    int Hthresh = 0;
    int Vthresh = 0;
    cvCreateTrackbar("hue thresh", "result", &Hthresh, 10000, NULL);
    cvCreateTrackbar("value thresh", "result", &Vthresh, 100, NULL);
    IplImage* h_plane ;
    IplImage* s_plane ;
    IplImage* v_plane ;
	for(;;)
	{
		frame = cvQueryFrame( capture );
		if(frame){
		  cvSmooth(frame, frame, CV_GAUSSIAN, 25, 25, 0, 0);
		  cvSetImageROI(frame, cvRect(0,(frame->height/2),frame->width, (frame->height/2)));
		  
		  h_plane = cvCreateImage( cvGetSize(frame ), 8, 1 );
		  s_plane = cvCreateImage( cvGetSize( frame), 8, 1 );
		  v_plane = cvCreateImage( cvGetSize( frame ), 8, 1 );
		  imHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
		  cvCvtColor(frame,imHSV,CV_BGR2HSV);
		  cvCvtPixToPlane( imHSV , h_plane, s_plane, v_plane, 0 );
		  CvHistogram* hist_h,*hist_v;
		  int h_bins = H_BINS, v_bins = V_BINS;
		  
		  hist_h = create_histogram(h_plane, H_MAX, &h_bins);
		  hist_v = create_histogram(v_plane, V_MAX, &v_bins);
		  
		  if(process_frame(frame, h_plane, v_plane, hist_h, hist_v, Hthresh, Vthresh)==-1)
		    break;
		}
		cvReleaseImage(&imHSV );
		cvReleaseImage(&h_plane );
		cvReleaseImage(&v_plane );
	}

	cvReleaseCapture( &capture );
	cvDestroyWindow("Display");

	//  cvDestroyWindow("FinalDisplay");
	cvDestroyWindow("VDisplay");
	cvDestroyWindow("Result");
	cvDestroyWindow("Display1");
	return 0;
}


CvHistogram* create_histogram(IplImage* plane, int range, int* bins)
{
	
	float ranges[] = { 0, range };
	float* temp_range[] = { ranges };
	CvHistogram* hist = cvCreateHist( 1, bins, CV_HIST_ARRAY, temp_range, 1 );
	cvCalcHist(&plane, hist, 0, 0 ); // Compute histogram
	return hist;
}

int process_frame(IplImage* frame, IplImage* h_plane, IplImage* v_plane, CvHistogram* hist_h, CvHistogram* hist_v, int Hthresh, int Vthresh)
{
	IplImage *resultF;
	int c;

	if(!frame) return -1;


	resultF = cvCreateImage( cvGetSize(frame), frame->depth, frame->nChannels );
	cvCopy(frame,resultF,NULL);

	cvShowImage("Display1", frame );

	//cvShowImage("Display", frame );
	//cvShowImage("Display", frame );

	IplImage* result = cvCreateImage( cvGetSize(h_plane ), 8, 1 );
	int width = frame->width;
	int height = frame->height;

	int h_bins = H_BINS, v_bins = V_BINS;

	int *h_s;
	h_s = malloc(sizeof(int)*h_bins);
	int *v_s;
	v_s = malloc(sizeof(int)*v_bins);

	int j;
	float pr = (float)Hthresh/100000;
	pr = pr*result->width*result->height;
	
	for(j = 0 ; j < h_bins ; j++)
	{
	  if(cvQueryHistValue_1D(hist_h,j) < pr)
	    {
	      h_s[j] = 0;//is obstical
	    }
	  else
	    {
	      h_s[j] = H_MAX;
	
	    }
	}
	pr = (float)Vthresh/1000;
	pr = pr*result->width*result->height;
	

	for(j = 0 ; j < v_bins ; j++)
	  {
	    if(cvQueryHistValue_1D(hist_v,j) < pr)
	      {
		//	printf("%f %f\n",cvQueryHistValue_1D(hist_v,j),pr);
		v_s[j] = 0;
	      }
	    else
	      v_s[j] = V_MAX;
	  	    //	    printf("%f %d %d\n",cvQueryHistValue_1D(hist_v,j),Vthresh,v_s[j]);
	    
	  }
	//	getchar();
	int i;
	CvScalar Black, White;
	Black.val[0] = 0;
	White.val[0] = 255;
	int p,q;
	cvCopy(v_plane, result,NULL);
	int h_bsize = H_MAX/H_BINS;
	int v_bsize = V_MAX/V_BINS;
	//printf("%d %d\n",h_bsize,v_bsize);
	for(i = 0 ; i < result->height ; i++)
	{
		for(j = 0 ; j < result->width ; j++)
		{
			CvScalar s;
			s = cvGet2D(h_plane ,i ,j);
			if(h_s[(int)s.val[0]/(h_bsize)]  != 0 )//obsticals are white
				cvSet2D(h_plane,i,j,Black);
			else
				cvSet2D(h_plane,i,j,White);


			s = cvGet2D(v_plane ,i ,j);
			if(s.val[0] == 255)
			  s.val[0] = 254;
			if(v_s[(int)s.val[0]/(v_bsize)]  != 0)
				cvSet2D(v_plane,i,j,Black);
			else
			  {
			    cvSet2D(v_plane,i,j,White);
			  }
		}
	}
	

	//for(i = 0; i < result->height; i++)
	//{
	//	for(j = 0; j < result->width; j++)
	//	{
	//		if(countArray[i/BLOCK_DIM][j/BLOCK_DIM] <= AMT_BLACK)
	//			cvSet2D(result, i, j, Black);			
	//		else
	//			cvSet2D(result, i, j, White);
	//	}
	//}

	cvOr(v_plane,h_plane,result,NULL);

	cvShowImage("Result", result );
	cvShowImage("HDisplay", h_plane );
	cvShowImage("VDisplay", v_plane );
	//UNCOMMENT FOR CONTOUR
	/*
	CvMemStorage* memStorage = cvCreateMemStorage(0);
	CvSeq* contours = 0;
	cvFindContours(result, memStorage, &contours,sizeof(CvContour),CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	CvSeq* co;
	for( co = contours; co != NULL; co = co->h_next) 
	{
		cvDrawContours(resultF,co,cvScalarAll(0),cvScalarAll(0),-1,5,8,cvPoint(0,0));
	}
	cvShowImage("FinalDisplay", resultF );
	cvReleaseMemStorage(&memStorage);
	*/
	//comment stops here
	cvReleaseImage(&result );
	cvReleaseImage(&resultF );
	free(h_s);
	free(v_s);
	c = cvWaitKey(30);
	if( c == 'q' || c == 'Q' || (c & 100) == 27 )
		return -1;
	return 0;
}

/*llnode* single_frame_detect_obstacles()
{
	CvCapture* capture = 0;
	capture = cvCaptureFromCAM(0);
	IplImage *frame, *imHSV;
	
	frame = cvQueryFrame( capture );
	cvSmooth(frame, frame, CV_GAUSSIAN, 25, 25, 0, 0);

	IplImage* h_plane = cvCreateImage( cvGetSize(frame ), 8, 1 );
	IplImage* s_plane = cvCreateImage( cvGetSize( frame), 8, 1 );
	IplImage* v_plane = cvCreateImage( cvGetSize( frame ), 8, 1 );
	imHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvCvtColor(frame,imHSV,CV_BGR2HSV);
	cvCvtPixToPlane( imHSV , h_plane, s_plane, v_plane, 0 );
	CvHistogram* hist_h,*hist_v;
	int h_bins = H_BINS, v_bins = V_BINS;
	hist_h = create_histogram(h_plane, H_MAX, &h_bins);
	hist_v = create_histogram(v_plane, V_MAX, &v_bins);

	llnode * head = process_frame(frame, h_plane, v_plane, hist_h, hist_v);

	cvReleaseImage(&imHSV );
	cvReleaseImage(&h_plane );
	cvReleaseImage(&v_plane );
	
	cvReleaseCapture( &capture );
	cvDestroyWindow("Display");

	//  cvDestroyWindow("FinalDisplay");
	cvDestroyWindow("VDisplay");
	cvDestroyWindow("Result");
	cvDestroyWindow("Display1");

	return head;
}
*/
