# Desc: Plugin for IMU_LSM303DLH
# CVS: $Id$

all: libimulsm303dlh.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialFixedMsg.o: ../driverlib/SerialFixedMsg.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialDevice.o: ../driverlib/SerialDevice.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libimulsm303dlh.so: imulsm303dlh.o imulsm303dlhSerial.o SerialDevice.o SerialFixedMsg.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
