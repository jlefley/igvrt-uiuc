# Desc: Plugin for IMU_FieldForce_TCM_XB
# CVS: $Id$

all: libimufieldforcetcmxb.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $<

libimufieldforcetcmxb.so: imufieldforcetcmxb.o
	$(CXX) -shared -o $@ $^

clean:
	rm -f *.o *.so
