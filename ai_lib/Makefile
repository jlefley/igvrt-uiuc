OBJECTS = ai_lib_test.o
CC = g++
LINK_OPENCV = -lcxcore -lcv -lcvaux -lhighgui

ai_lib_test: $(OBJECTS)
	$(CC) $(OBJECTS) -o ai_lib_test

stereo_demo: stereo.o stereo_demo.cpp
	$(CC) -c -g stereo_demo.cpp -o stereo_demo.o
	$(CC) stereo.o stereo_demo.o -g $(LINK_OPENCV) -o stereo_demo
	
stereo.o: stereo.cpp stereo.h
	$(CC) -c -g stereo.cpp
	
obstacle_detection.o: obstacle_detection.cpp obstacle_detection.h
	$(CC) -c obstacle_detection.cpp -g -o obstacle_detection.o
	
obstacle_demo: obstacle_detection.o obstacle_detection_demo.cpp
	$(CC) -c obstacle_detection_demo.cpp -o obstacle_detection_demo.o
	$(CC) obstacle_detection.o obstacle_detection_demo.o -g $(LINK_OPENCV) -o obstacle_demo
	
lines.o: lines.cpp lines.h
	$(CC) -c lines.cpp -g -o lines.o
	
lines_demo: lines.o lines_demo.cpp
	$(CC) -c lines_demo.cpp -g -o lines_demo.o
	$(CC) lines.o lines_demo.o -g $(LINK_OPENCV) -o lines_demo
	
test: stereo_demo obstacle_demo

clean:
	-rm $(OBJECTS) ai_lib_test stereo_demo stereo.o *.o


