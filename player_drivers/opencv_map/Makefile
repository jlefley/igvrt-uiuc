# Desc: Plugin for opencv_map
# CVS: $Id$

all: libopencvmap.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libopencvmap.so: opencvmap.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
