# Desc: Plugin for Roomba
# CVS: $Id$

all: libroomba_igvrt.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialFixedMsg.o: ../driverlib/SerialFixedMsg.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialDevice.o: ../driverlib/SerialDevice.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

Kalman.o: ../driverlib/Kalman.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

DiffSteering.o: ../driverlib/DiffSteering.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libroomba_igvrt.so: roomba_igvrt.o Roomba.o SerialFixedMsg.o SerialDevice.o Kalman.o DiffSteering.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
