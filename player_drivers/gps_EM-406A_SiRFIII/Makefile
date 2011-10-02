# Desc: Plugin for GPS_EM-406A_SiRFIII
# CVS: $Id$

all: libgpsem406asirfiii.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialFixedMsg.o: ../driverlib/SerialFixedMsg.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialDevice.o: ../driverlib/SerialDevice.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libgpsem406asirfiii.so: gpsem406asirfiii.o gpsem406asirfiii_serial.o SerialFixedMsg.o SerialDevice.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
