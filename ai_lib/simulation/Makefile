CXX = g++
CXXFLAGS = `pkg-config --cflags playerc++`
LDFLAGS = `pkg-config --libs playerc++`

all: sonarobstacleavoid

single: sonarobstacleavoid.cc args.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f sonarrobstacleavoid 
