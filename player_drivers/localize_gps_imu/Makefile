# Desc: Plugin for Localize_GPS_IMU
# CVS: $Id$

all: liblocalizegpsimu.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

liblocalizegpsimu.so: localizegpsimu.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
