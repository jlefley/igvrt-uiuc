import sys
sys.path.append('/usr/local/lib64/python2.6/site-packages')
import stereo
from opencv import highgui,cv

font = cv.cvInitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0)

def depthmatch(x,y,leftimage,rightimage,roi=20,buf=10,debug=False):
    __doc__ = """depthmatch function
    x,y : (int) pixel position of target in left image
    leftimage, rightimage : (IplImage) stereo images
    roi: (int) region of interest around x,y to use in matching
    buf: (int) buffer outside of a straight horizontal search for a match
    """
    info = cv.cvGetSize(leftimage)
    width = info.width
    height = info.height

    (y1,x1,y2,x2) = (y-roi,x-roi,y+roi,x+roi)
    #template = cv.cvCreateImage((roi*2,roi*2), 8, 3)
    if y1<0: y1 = 0
    if x1<0: x1 = 0
    if y2>height: y2 = height
    if x2>width: x2 = width
    #cv.cvSetZero(template)
    # copy subregion roi x roi

    template_rect = cv.cvRect(x1,y1,(x2-x1),(y2-y1))
    template = cv.cvGetSubRect(leftimage, template_rect)
    (y3,x3,y4,x4) = (y-roi-buf,x-roi-buf,y+roi+buf,width) # +/- 20 pixels in vertical direction, -20 to the right edge
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
    depth = stereo.depth(x, x3+min_point.x, max_pixels=width/2)
    
    if debug:
        print "Input image: %ix%i, target: (%i,%i)" % (width,height,x,y)
        print "Template box: (%i,%i) to (%i,%i)" % (x1, y1, x2, y2)
        print "Search area: (%i,%i) to (%i,%i)" % (x3, y3, x4, y4)
        print "%ix%i, %ix%i" % (W,H,w,h)
        print "Result matrix %ix%i" % (resx, resy)
        print "stereo.depth(%i,%i,max_pixels=%i)" % (x, min_point.x+x3,width/2)
        if depth[0]:
            print "Depth: ", depth[0], "(cm)"
        #cv.cvRectangle(rightimage, cv.cvPoint(x1,y1), cv.cvPoint(x2,y2), (255,0,0))
        cv.cvRectangle(rightimage, cv.cvPoint(min_point.x+x3,min_point.y+y3), cv.cvPoint(min_point.x+x3+roi*2,min_point.y+y3+roi*2), (0,255,0))
        cv.cvRectangle(rightimage, cv.cvPoint(x3,y3), cv.cvPoint(x4,y4), (0,0,255))
        cv.cvRectangle(leftimage, cv.cvPoint(x1,y1), cv.cvPoint(x2,y2), (255,0,0))
        #cv.cvRectangle(leftimage, cv.cvPoint(min_point.x+x3,min_point.y+y3), cv.cvPoint(min_point.x+x3+roi*2,min_point.y+y3+roi*2), (0,255,0))
        cv.cvRectangle(leftimage, cv.cvPoint(x3,y3), cv.cvPoint(x4,y4), (0,0,255))
        if depth[0]:
            cv.cvPutText(leftimage, "%5f(cm)" % depth[0], (x1,y1), font, (255,255,255))
        highgui.cvShowImage("depthmatch - template", template)
        highgui.cvShowImage("depthmatch - match", resultmat)
        highgui.cvShowImage("depthmatch - right", rightimage)
        highgui.cvShowImage("depthmatch - left", leftimage)
        
if len(sys.argv) == 3:
    left = highgui.cvLoadImage(str(sys.argv[1]))
    right = highgui.cvLoadImage(str(sys.argv[2]))
    from_file = True
else:
    from_file = False

if __name__ == "__main__" and from_file:
    depthmatch(720, 1024/2, left, right, buf=600, debug=True) 
    while True:
        
        highgui.cvWaitKey(10)
        
if __name__ == "__main__" and not from_file:
    highgui.cvNamedWindow("depthmatch - right")
    highgui.cvNamedWindow("depthmatch - left")
    highgui.cvNamedWindow("depthmatch - match")
    highgui.cvNamedWindow("depthmatch - template")
    xmatch = 0
    ymatch = 0
    def mousecb(e,x,y,f,p):
        global xmatch, ymatch
        if highgui.CV_EVENT_LBUTTONDOWN == e:
            xmatch = x
            ymatch = y
    highgui.cvSetMouseCallback("depthmatch - left", mousecb)
    cleft = highgui.cvCreateCameraCapture(2)
    cright = highgui.cvCreateCameraCapture(1)
    size = cv.cvGetSize(highgui.cvQueryFrame(cleft))
    xmatch = size.width / 2 + 1
    ymatch = size.height / 2 + 1
    while True:
        highgui.cvGrabFrame(cleft)
        highgui.cvGrabFrame(cright)
        left = highgui.cvRetrieveFrame(cleft)
        right = highgui.cvRetrieveFrame(cright)
        depthmatch(xmatch, ymatch, left, right, buf=50, debug=True)
        highgui.cvWaitKey(10)
    """
IMGW = 1280 # 640
IMGH = 1024 # 480 

stereostate_bm = cv.cvCreateStereoBMState(cv.CV_STEREO_BM_BASIC, 0)
stereostate_gc = cv.cvCreateStereoGCState(16, 3)
if not from_file:
    cap0 = highgui.cvCreateCameraCapture(1)
    highgui.cvSetCaptureProperty(cap0, highgui.CV_CAP_PROP_FRAME_WIDTH, IMGW)
    highgui.cvSetCaptureProperty(cap0, highgui.CV_CAP_PROP_FRAME_HEIGHT, IMGH)
    cap1 = highgui.cvCreateCameraCapture(2)
    highgui.cvSetCaptureProperty(cap1, highgui.CV_CAP_PROP_FRAME_WIDTH, IMGW)
    highgui.cvSetCaptureProperty(cap1, highgui.CV_CAP_PROP_FRAME_HEIGHT, IMGH)

disparity = cv.cvCreateMat(IMGH,IMGW, cv.CV_16S)
disparity_right = cv.cvCreateMat(IMGH,IMGW, cv.CV_16S)
disparity_image = cv.cvCreateMat(IMGH, IMGW, cv.CV_8U)
joined = cv.cvCreateImage(cv.cvSize(IMGW,IMGH), 8, 3)

highgui.cvNamedWindow("Disparity")

tx = 0
ty = 0

bm = False # use this to switch between bm (faster) and gc (better) stereo algos

def mousecb(event,x,y,flag,param):
    if event == highgui.CV_EVENT_RBUTTONDOWN:
        global tx,ty
        tx = x
        ty = y
        print x, y
        
highgui.cvSetMouseCallback("Disparity", mousecb, 0)


while True:
    if not from_file:
        highgui.cvGrabFrame(cap0)
        highgui.cvGrabFrame(cap1)
        left = highgui.cvRetrieveFrame(cap0)
        right = highgui.cvRetrieveFrame(cap1)
    if not left or not right:
        print "Cap failed"
        break
        
    if bm:
        cv.cvSetZero(disparity)
        cv.cvSetZero(disparity_right)
    cv.cvSetZero(disparity_image)
    cv.cvSetZero(joined)

    left_bw = cv.cvCreateImage((IMGW,IMGH),8,1)
    right_bw = cv.cvCreateImage((IMGW,IMGH),8,1)
    cv.cvSetZero(left_bw)
    cv.cvSetZero(right_bw)
    highgui.cvConvertImage(left,left_bw)
    highgui.cvConvertImage(right,right_bw)
    cv.cvEqualizeHist(left_bw, left_bw)
    cv.cvEqualizeHist(right_bw, right_bw)
    
    
    if bm:
        cv.cvFindStereoCorrespondenceBM(left_bw, right_bw, disparity, stereostate_bm)
        cv.cvConvertScale(disparity, disparity_image, 255/1008.0)
    else:
        cv.cvFindStereoCorrespondenceGC(left_bw, right_bw, disparity, disparity_right, stereostate_gc, 0)
        cv.cvConvertScale(disparity, disparity_image, -16.0)

    # map onto image
    #leftx = cv.cvGet2D(disparity, tx, ty)
    
    highgui.cvShowImage("Disparity", disparity_image)
    highgui.cvWaitKey(10)
"""
