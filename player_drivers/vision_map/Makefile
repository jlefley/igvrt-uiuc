# Desc: Plugin for vision_map
# CVS: $Id$

all: libvisionmap.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libvisionmap.so: visionmap.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
