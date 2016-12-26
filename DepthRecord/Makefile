CFLAGS = -std=c++11 -O2

all: DepthRecord dvsPlayer

DepthRecord: DepthRecord.cpp
	g++ $(CFLAGES) -o DepthRecord DepthRecord.cpp -lOpenNI2 `pkg-config opencv --cflags --libs`

dvsPlayer: dvsPlayer.cpp
	g++ $(CFLAGES) -o dvsPlayer dvsPlayer.cpp `pkg-config opencv --cflags --libs`

clean:
	rm -f DepthRecord
	rm -f dvsPlayer
