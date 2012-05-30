import sys
sys.path.append('/usr/local/lib64/python2.6/site-packages')
from opencv import highgui,cv
import math

mask = highgui.cvLoadImage(str(sys.argv[1]))


cap = highgui.cvCreateCameraCapture(1)
IMGW = 640
IMGH = 400
highgui.cvSetCaptureProperty(cap, highgui.CV_CAP_PROP_FRAME_WIDTH, IMGW)
highgui.cvSetCaptureProperty(cap, highgui.CV_CAP_PROP_FRAME_HEIGHT, IMGH)
tmp = highgui.cvQueryFrame(cap)
# resize mask
size = cv.cvGetSize(tmp)
mask_r = cv.cvCreateImage(size, 8, 3)
cv.cvResize(mask, mask_r)
mask_bw = cv.cvCreateImage(size, 8, 1)
cv.cvCvtColor(mask_r, mask_bw, cv.CV_RGB2GRAY)
total_pixels = size.width*size.height
sample_pixels = 0.0
for x in xrange(size.width):
    for y in xrange(size.height):
        if cv.cvGetReal2D(mask_bw, y, x) > 0:
            sample_pixels = sample_pixels + 1
print "Sample region: %f%%" % (100*sample_pixels/total_pixels)
del(tmp)


h_bins = 20
h_limit = 180
s_bins = 32
v_bins = 32
v_limit = 255
# create histogram with 30 bins
h_hue = cv.cvCreateHist([h_bins], cv.CV_HIST_ARRAY, [[0,h_limit]], 1)
h_sat = cv.cvCreateHist([s_bins], cv.CV_HIST_ARRAY, [[0,255]], 1)
h_val = cv.cvCreateHist([v_bins], cv.CV_HIST_ARRAY, [[0,v_limit]], 1)

# histogram limits
hsv_min = cv.cvScalar(0, 30, 10, 0)
hsv_max = cv.cvScalar(180, 256, 256, 0)
scalewidth = 20
scaleheight = 400.0

# object classification boundries
# if the bin value a given pixel falls into is lower than one of these cutoffs,
# that pixel is classified as an obstacle
hue_cutoff = 0.01*sample_pixels  # 1.0%
val_cutoff = 0.015*sample_pixels # 1.5%
def cb_hue(v):
    global hue_cutoff
    hue_cutoff = v

def cb_val(v):
    global val_cutoff
    val_cutoff = v
    
# windows
highgui.cvNamedWindow("Input")
highgui.cvNamedWindow("Histogram - Hue")
highgui.cvCreateTrackbar("Threshold", "Histogram - Hue", hue_cutoff, int(sample_pixels), cb_hue)
highgui.cvNamedWindow("Histogram - Value")
highgui.cvCreateTrackbar("Threshold", "Histogram - Value", val_cutoff, int(sample_pixels), cb_val)
highgui.cvNamedWindow("Obstacles")

highgui.cvMoveWindow("Input", 0, 0)
highgui.cvMoveWindow("Histogram - Hue", 0, size.height + 75)
highgui.cvMoveWindow("Histogram - Value", int(h_bins*scalewidth) + 25, size.height + 75)
highgui.cvMoveWindow("Obstacles", size.width + 25, 0)
def hsv2rgb (hue):
    # convert the hue value to the corresponding rgb value

    sector_data = [[0, 2, 1],
                   [1, 2, 0],
                   [1, 0, 2],
                   [2, 0, 1],
                   [2, 1, 0],
                   [0, 1, 2]]
    hue *= 0.1 / 3
    sector = cv.cvFloor (hue)
    p = cv.cvRound (255 * (hue - sector))
    if sector & 1:
        p ^= 255

    rgb = {}
    rgb [sector_data [sector][0]] = 255
    rgb [sector_data [sector][1]] = 0
    rgb [sector_data [sector][2]] = p

    return cv.cvScalar (rgb [2], rgb [1], rgb [0], 0)

img_h = cv.cvCreateImage(size, 8, 1)
img_s = cv.cvCreateImage(size, 8, 1)
img_v = cv.cvCreateImage(size, 8, 1)
thresh_mask = cv.cvCreateImage(size, 8, 1)
hist_hue_img = cv.cvCreateImage((int(h_bins*scalewidth),scaleheight),8,3)
hist_val_img = cv.cvCreateImage((int(v_bins*scalewidth),scaleheight),8,3)
output_mask = cv.cvCreateImage(size, 8, 1)

while True:
    img = highgui.cvQueryFrame(cap)
    cv.cvZero(img_h)
    cv.cvZero(img_s)
    cv.cvZero(img_v)
    cv.cvZero(thresh_mask)
    highgui.cvShowImage("Input", img)
    # 5x5 Gaussian Blur
    cv.cvSmooth(img, img, cv.CV_GAUSSIAN, 5, 5)
    # convert to HSV
    cv.cvCvtColor(img, img, cv.CV_BGR2HSV)

    # threshold bad values
    cv.cvInRangeS(img, hsv_min, hsv_max, thresh_mask)
    
    cv.cvAnd(thresh_mask, mask_bw, thresh_mask)
    # Hue(0,180), Saturation(0,255), Value(0,255)
    cv.cvSplit(img, img_h, img_s, img_v, 0)
    
    # calculate histogram
    cv.cvCalcHist(img_h, h_hue, 0, thresh_mask)
    cv.cvCalcHist(img_s, h_sat, 0, thresh_mask)
    cv.cvCalcHist(img_v, h_val, 0, thresh_mask)
    
    # Don't normalize, use total mask pixels to calculate relative importance
    #cv.cvNormalizeHist(h_hue, 180)
    #cv.cvNormalizeHist(h_sat, 255)
    #cv.cvNormalizeHist(h_val, 255)
    #minv,maxv,minp,maxp = cv.cvMinMaxLoc(img_h)
    #print minv,maxv


    cv.cvZero(hist_hue_img)
    #hue_min,hue_max,min_loc,max_loc = cv.cvGetMinMaxHistValue(h_hue)
    for h in xrange(h_bins):
        hue = cv.cvGetReal1D(h_hue.bins, h)
        color = hsv2rgb(h*h_limit/h_bins)
        cv.cvRectangle(hist_hue_img, (h*scalewidth,0), ((h+1)*scalewidth, (hue/sample_pixels)*scaleheight), color, cv.CV_FILLED)   
    cv.cvLine(hist_hue_img, (0, scaleheight*hue_cutoff/sample_pixels), (h_bins*scalewidth, scaleheight*hue_cutoff/sample_pixels), (255,0,0), 1)
    highgui.cvShowImage("Histogram - Hue", hist_hue_img)
    

    cv.cvZero(hist_val_img)
    #val_min,val_max,min_loc,max_loc = cv.cvGetMinMaxHistValue(h_val)
    for v in xrange(v_bins):
        val = cv.cvGetReal1D(h_val.bins, v)
        color = cv.cvScalar(180, 255, v*v_limit/v_bins)
        cv.cvRectangle(hist_val_img, (v*scalewidth,0), ((v+1)*scalewidth, (val/sample_pixels)*scaleheight), color, cv.CV_FILLED)   
    cv.cvLine(hist_val_img, (0, scaleheight*val_cutoff/sample_pixels), (v_bins*scalewidth, scaleheight*val_cutoff/sample_pixels), (255,0,0), 1)
    highgui.cvShowImage("Histogram - Value", hist_val_img)
    
    # classify objects
    cv.cvZero(output_mask)
    for x in xrange(size.width):
        for y in xrange(size.height):
            hue = cv.cvGetReal2D(img_h, y, x)
            hue_bin = math.ceil(hue*h_bins/h_limit)-1
            if hue_bin < 0: hue_bin = 0
            #print hue_bin
            if cv.cvGetReal1D(h_hue.bins, int(hue_bin)) < hue_cutoff:
                cv.cvSetReal2D(output_mask, y, x, 255)
                continue
            val = cv.cvGetReal2D(img_v, y, x)
            val_bin = math.ceil(val*v_bins/v_limit)-1
            if val_bin < 0: val_bin = 0
            if cv.cvGetReal1D(h_val.bins, int(val_bin)) < val_cutoff:
                cv.cvSetReal2D(output_mask, y, x, 255)
                continue
        #highgui.cvWaitKey(1)

    highgui.cvShowImage("Obstacles", output_mask)
    highgui.cvWaitKey(10)
