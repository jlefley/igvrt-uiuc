# Desc: Plugin for Motor Controller
# CVS: $Id$

all: libmotorcontroller.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialFixedMsg.o: ../driverlib/SerialFixedMsg.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialDevice.o: ../driverlib/SerialDevice.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libmotorcontroller.so: motorcontroller.o MotorControllerSerial.o SerialFixedMsg.o SerialDevice.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
