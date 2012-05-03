APP = igvrt
CC = g++

all: $(APP).cc
	$(CC) -o $(APP) `pkg-config --cflags playerc++` $(APP).cc `pkg-config --libs playerc++`

clean:
	rm -f *.o $(APP)

