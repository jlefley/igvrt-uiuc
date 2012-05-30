# right click on a location to see HSV thresholding result, left click anywhere to return to RGB image

import sys
sys.path.append('/usr/local/lib64/python2.6/site-packages')
import stereo
from opencv import highgui,cv



if len(sys.argv) >= 2:
    imgorig = highgui.cvLoadImage(sys.argv[1])
    img = cv.cvCloneImage(imgorig)
    cv.cvCvtColor(img,img,cv.CV_RGB2HSV)
    size = cv.cvGetSize(img)
    IMGW = size.width
    IMGH = size.height
    fromfile = True
else:
    fromfile = False
    IMGW = 640
    IMGH = 480
    cap0 = highgui.cvCreateCameraCapture(0)
    highgui.cvSetCaptureProperty(cap0, highgui.CV_CAP_PROP_FRAME_WIDTH, IMGW)
    highgui.cvSetCaptureProperty(cap0, highgui.CV_CAP_PROP_FRAME_HEIGHT, IMGH)

joined = cv.cvCreateImage(cv.cvSize(IMGW,IMGH), 8, 3)

highgui.cvNamedWindow("Blob")

tx = 100
ty = 100
showoriginal = True
def mousecb(event,x,y,flag,param):
    global tx,ty,showoriginal
    if event == highgui.CV_EVENT_RBUTTONDOWN:
        
        tx = x
        ty = y
        print x, y
        showoriginal = False
    elif event == highgui.CV_EVENT_LBUTTONDOWN:
        showoriginal = True
        print "Orig"
highgui.cvSetMouseCallback("Blob", mousecb, 0)

while True:
    if not fromfile:
        img = highgui.cvQueryFrame(cap0)
        cv.cvCvtColor(img,img,cv.CV_RGB2HSV)
    (h,s,v) = (img[ty][tx][0],img[ty][tx][1],img[ty][tx][2])
    thresh = cv.cvCreateImage(cv.cvSize(IMGW,IMGH),8,1)
    cv.cvSetZero(thresh)
    
    # dynamic floodfill
    import queue
    q = queue.Queue(0)
    q.put({"x":tx,"y":ty})
    thresh[ty][tx] = 255
    while True:
        # move left and right until boundary is hit
        n = q.pop()
        l = n.x - 1
        r = n.x + 1
        y = n.y
        while True:
            if h-20 < img[y][l][0] < h+20:
                thresh[y][l] = 255
                #
            else:
                break
            l = l - 1
        #while True:
    #cv.cvInRangeS(img, (h-10,s-40,v-20,0), (h+10,s+40,v+20,0), thresh)
    result = cv.cvCreateImage(cv.cvSize(IMGW,IMGH),8,3)
    cv.cvSetZero(result)
    
    cv.cvOr(img,img,result,thresh)
    if showoriginal:
        highgui.cvShowImage("Blob", imgorig)
    else:
        highgui.cvShowImage("Blob", result)
    c = highgui.cvWaitKey(10)
