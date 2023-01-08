SRC := $(wildcard *.cpp)
CFLAGS = -I. -Wall -std=c++11

all: cutSphereMove

cutSphereMove: $(SRC)
	g++ -o $@ $^ $(CFLAGS)

clean:
	rm cutSphereMove
