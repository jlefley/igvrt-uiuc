APP = opencv_homographic_calib
CC = g++

all: $(APP).cc
	$(CC) -o $(APP) `pkg-config --cflags playerc++` $(APP).cc `pkg-config --libs playerc++` -I /usr/include/opencv -L /usr/lib -lm -lopencv_core -lopencv_highgui -lopencv_imgproc

clean:
	rm -f *.o $(APP)

