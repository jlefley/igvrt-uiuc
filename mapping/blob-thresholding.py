# threshold value testing
import sys
sys.path.append('/usr/local/lib64/python2.6/site-packages')
from opencv import cv,highgui
import time

h = 0
s = 0
v = 0
h2=255
s2=255
v2=255
d = 3
e = 1
IMGW = 640
IMGH = 480

def tb_h(value):
    global h
    h = value
    
def tb_s(value):
    global s
    s = value
    
def tb_v(value):
    global v
    v = value

def tb_d(value):
    global d
    d = value
    
def tb_h2(value):
    global h2
    h2 = value
    
def tb_s2(value):
    global s2
    s2 = value
    
def tb_v2(value):
    global v2
    v2 = value

def tb_d2(value):
    global d2
    d2 = value

def tb_e(value):
    global e
    e = value
    

def main(): # ctrl+c to end
    global h,s,v,h2,v2,s2,d,e
    highgui.cvNamedWindow("Camera 1", 1)
    highgui.cvNamedWindow("Orig", 1)
    highgui.cvCreateTrackbar("H", "Camera 1", h, 256, tb_h)
    highgui.cvCreateTrackbar("S", "Camera 1", s, 256, tb_s)
    highgui.cvCreateTrackbar("V", "Camera 1", v, 256, tb_v)
    highgui.cvCreateTrackbar("H2", "Camera 1", h2, 256, tb_h2)
    highgui.cvCreateTrackbar("S2", "Camera 1", s2, 256, tb_s2)
    highgui.cvCreateTrackbar("V2", "Camera 1", v2, 256, tb_v2)
    highgui.cvCreateTrackbar("Dilate", "Camera 1", d, 30, tb_d)
    highgui.cvCreateTrackbar("Erode", "Camera 1", e, 30, tb_e)
    
    cap = highgui.cvCreateCameraCapture(1)
    highgui.cvSetCaptureProperty(cap, highgui.CV_CAP_PROP_FRAME_WIDTH, IMGW)
    highgui.cvSetCaptureProperty(cap, highgui.CV_CAP_PROP_FRAME_HEIGHT, IMGH)
    c = 0
    t1 = tdraw = time.clock()
    t = 1
    font = cv.cvInitFont(cv.CV_FONT_HERSHEY_PLAIN, 1, 1)
    while c != 0x27:
        image = highgui.cvQueryFrame(cap)
        if not image:
            print "capture failed"
            break
            
        thresh = cv.cvCreateImage(cv.cvSize(IMGW,IMGH),8,1)
        cv.cvSetZero(thresh)
        cv.cvCvtColor(image,image,cv.CV_RGB2HSV)
        cv.cvInRangeS(image, (h,s,v,0), (h2,s2,v2,0), thresh)
        result = cv.cvCreateImage(cv.cvSize(IMGW,IMGH),8,3)
        cv.cvSetZero(result)
        
        cv.cvOr(image,image,result,thresh)
        for i in range(1,e):
            cv.cvErode(result,result)
        for i in range(1,d):
            cv.cvDilate(result,result)
            
        # floodfill objects back in, allowing threshold differences outwards
        
        t2 = time.clock()
        if t2 > tdraw+0.3:
            t = t2-t1
            tdraw=t2
        cv.cvPutText(result, "FPS: " + str(1 / (t)), (0,25), font, (255,255,255))
        t1 = t2
        highgui.cvShowImage("Orig", image)
        highgui.cvShowImage("Camera 1", result)
        c = highgui.cvWaitKey(10)
main()
