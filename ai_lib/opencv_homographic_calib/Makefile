APP = opencv_homographic_calib
CC = g++

all: $(APP).cc
	$(CC) -o $(APP) `pkg-config --cflags playerc++` $(APP).cc `pkg-config --libs playerc++` -I /usr/include/opencv -L /usr/lib -lm -lcv -lhighgui -lcvaux

clean:
	rm -f *.o $(APP)

