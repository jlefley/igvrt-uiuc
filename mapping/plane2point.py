import math
import sys
sys.path.append('/usr/local/lib64/python2.6/site-packages')
import stereo
from opencv import highgui,cv

def plane2point(leftx, lefty, rightx, righty, baseline=2.7, focal_length=100):
    """Returns (X,Y,Z), with (0,0,0) = the lens of the left camera. Pinhole model."""
    if (leftx == rightx): return (0,0,0)
    z = (baseline * focal_length) / (leftx - rightx)
    x = leftx * z / focal_length
    y = lefty * z / focal_length
    return (x,y,z)
    
def depthmatch(x,y,leftimage,rightimage,roi=80,buf=50,baseline=2.7,focal_length=80):
    """depthmatch function
    x,y : (int) pixel position of target in left image
    leftimage, rightimage : (IplImage) stereo images
    roi: (int) region of interest around x,y to use in matching
    buf: (int) buffer outside of a straight horizontal search for a match
    """
    #print "Match",x,y
    info = cv.cvGetSize(leftimage)
    width = info.width
    height = info.height
    centerx = width/2
    centery = height/2
    
    (y1,x1,y2,x2) = (y-roi,x-roi,y+roi,x+roi)
    if y1<0: y1 = 0
    if x1<0: x1 = 0
    if y2>height: y2 = height
    if x2>width: x2 = width
    # copy subregion roi x roi

    template_rect = cv.cvRect(x1,y1,(x2-x1),(y2-y1))
    template = cv.cvGetSubRect(leftimage, template_rect)
    
    #(y3,x3,y4,x4) = (y-roi-buf,x-roi-buf,y+roi+buf,width) # +/- 20 pixels in vertical direction, -20 to the right edge

    (y3,x3,y4,x4) = (y-roi-buf,0,y+roi+buf,x+roi+buf) # +/- buf pixels in vertical direction, +buf to the left edge
    if x3<0: x3 = 0
    if y3<0: y3 = 0
    if x4>=width: x4 = width-1
    if y4>height: y4 = height
    #cv.cvSetImageROI(rightimage, (y3,x3,y4,x4))

    rightsub_rect = cv.cvRect(x3,y3,(x4-x3),(y4-y3))
    rightsub = cv.cvGetSubRect(rightimage, rightsub_rect)
    # result matrix should be (W - w + 1) x (H - h + 1) where WxH are template dimensions, wxh are rightsub dimensions
    W = x4-x3
    H = y4-y3
    w = x2-x1
    h = y2-y1

    resy = (y4-y3)-(y2-y1)+1
    resx = (x4-x3)-(x2-x1)+1

    resultmat = cv.cvCreateImage((resx, resy), 32, 1)
    cv.cvZero(resultmat)
    # match template image in a subportion of rightimage
    cv.cvMatchTemplate(rightsub, template, resultmat, cv.CV_TM_SQDIFF)
    min_val, max_val, min_point, max_point = cv.cvMinMaxLoc(resultmat)
    cv.cvNormalize(resultmat, resultmat, 1, 0, cv.CV_MINMAX)
    depth = plane2point(x-centerx, y-centery, x3+min_point.x+roi-centerx, y3+min_point.y+roi-centery, baseline, focal_length)
    #print "Found match at", min_point.x+x3, min_point.y+y3
    return (depth, (x,y), (x3+min_point.x+roi, y3+min_point.y+roi))
    
def depthmatrix(leftimage, rightimage, precision=4, mask=0):
    """Returns a 3-channel 32bit floating-point distance matrix. Channels 1,2,3 = x,y,z coordinates of that point.
    Precision is the number of times to downsample mask. Downsample is the number of loops to 
    go through with successively smaller match areas. If mask is set, only pixels in the mask are set."""
    
    info = cv.cvGetSize(leftimage)
    width = info.width
    height = info.height
    precision_pixels = (2**precision)
    downsampled_size = cv.cvSize(width/precision_pixels, height/precision_pixels)
    print "Precision of", downsampled_size.width, downsampled_size.height, "px"
    if mask:
        downsampled_mask = cv.cvCreateImage(downsampled_size, 8, 1)
        cv.cvResize(mask, downsampled_mask)
    matx = cv.cvCreateImage(downsampled_size, 8, 1)
    maty = cv.cvCreateImage(downsampled_size, 8, 1)
    matz = cv.cvCreateImage(downsampled_size, 8, 1)
    for i in xrange(width/precision_pixels):
        for j in xrange(height/precision_pixels):
            if mask:
                if (not cv.cvGetReal2D(downsampled_mask, j, i)):
                    continue
            x = i*precision
            y = j*precision
            depth = depthmatch(x+precision_pixels/2, y+precision_pixels/2, leftimage, rightimage, roi=precision_pixels, buf=precision_pixels*2)
            #print i, j
            # fill in result matrix if mask wasn't 0 at this point (X,Y,Z)
            cv.cvSetReal2D(matx, j, i, int(depth[0][0]))
            cv.cvSetReal2D(maty, j, i, int(depth[0][1]))
            cv.cvSetReal2D(matz, j, i, int(depth[0][2]))
    return matz
    
test_number = 2
if __name__ == "__main__" and test_number == 1:
    left = highgui.cvLoadImage(str(sys.argv[1]))
    right = highgui.cvLoadImage(str(sys.argv[2]))
    size = cv.cvGetSize(left)
    #depth = depthmatrix(left, right)
    font = cv.cvInitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5)
    highgui.cvNamedWindow("depthmatch - left")
    highgui.cvNamedWindow("depthmatch - right")
    xmatch = 0
    ymatch = 0
    variable_roi = 20
    variable_buf = 10
    variable_focal = 100
    variable_base = 2.7
    def mousecb(e,x,y,f,p):
        global xmatch, ymatch
        if highgui.CV_EVENT_LBUTTONDOWN == e:
            xmatch = x
            ymatch = y
    def cb_roi(v):
        global variable_roi
        variable_roi = v
    def cb_buf(v):
        global variable_buf
        variable_buf = v
    def cb_focal(v):
        global variable_focal
        variable_focal=v
    def cb_base(v):
        global variable_base
        variable_base = v/10.0
    xmatch = size.width / 2 + 1
    ymatch = size.height / 2 + 1
    highgui.cvSetMouseCallback("depthmatch - left", mousecb)
    highgui.cvCreateTrackbar("ROI", "depthmatch - left", variable_roi, size.width, cb_roi)
    highgui.cvCreateTrackbar("Buffer", "depthmatch - left", variable_buf, size.width, cb_buf)
    highgui.cvCreateTrackbar("Focal Length", "depthmatch - left", variable_focal, 1000, cb_focal)
    highgui.cvCreateTrackbar("Baseline/10", "depthmatch - left", variable_base, 1000, cb_base)
    leftdraw = cv.cvCreateImage(size, 8, 3)
    rightdraw = cv.cvCreateImage(size, 8, 3)
    while 1:
        depth = depthmatch(xmatch, ymatch, left, right, roi=variable_roi, buf=variable_buf,baseline=variable_base, focal_length=variable_focal)
        cv.cvCopy(left, leftdraw)
        cv.cvCopy(right, rightdraw)
        cv.cvLine(leftdraw, depth[1], depth[2], (0,255,0), 2)
        cv.cvPutText(leftdraw, "%2f(m) at (%2f,%2f)" % (depth[0][2],depth[0][0],depth[0][1]), (xmatch,ymatch), font, (0,0,255))
        cv.cvLine(rightdraw, depth[2], depth[2], (0,0,255), 5)
        highgui.cvShowImage("depthmatch - left", leftdraw)
        highgui.cvShowImage("depthmatch - right", rightdraw)
        print depth
        highgui.cvWaitKey(10)
        
        
if __name__ == "__main__" and test_number == 2:
    left = highgui.cvLoadImage(str(sys.argv[1]))
    right = highgui.cvLoadImage(str(sys.argv[2]))
    highgui.cvNamedWindow("Depth")
    depth = depthmatrix(left, right, 4)
    depth_full = cv.cvCreateImage(cv.cvGetSize(left), 8, 1)
    cv.cvResize(depth, depth_full)
    highgui.cvShowImage("Depth", depth_full)
    while 1:
        highgui.cvWaitKey(10)
