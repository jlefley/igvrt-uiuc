# Desc: Plugin for IR/Sonar Prototype
# CVS: $Id$

all: libirsonarproto.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialFixedMsg.o: ../driverlib/SerialFixedMsg.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

SerialDevice.o: ../driverlib/SerialDevice.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libirsonarproto.so: irsonarproto.o IRSonarProtoDev.o SerialFixedMsg.o SerialDevice.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
