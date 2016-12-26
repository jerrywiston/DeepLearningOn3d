LNK_GL = -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor
CUDA_OPT = --gpu-architecture=compute_50 --gpu-code=compute_50
CFLAGS = -std=c++11 -O2

all: PointCloudOrien ViewPointCloud

PointCloudOrien: PointCloudOrien.cpp ransac.hpp orientation.hpp ransac.o
	g++ $(CFLAGS) -o PointCloudOrien PointCloudOrien.cpp ransac.o

ViewPointCloud: main.o  shader.o text2d.o skeleton3d.o cloud.o camera.o visibleObj.o ransac.o
	g++ $(CFLAGS) -fopenmp -o ViewPointCloud main.o shader.o text2d.o skeleton3d.o cloud.o camera.o visibleObj.o ransac.o $(LNK_GL)

main.o: main.cpp ransac.hpp orientation.hpp
	g++ $(CFLAGS) -c main.cpp $(LNK_GL)

shader.o: shader.cpp shader.hpp
	g++ $(CFLAGS) -c shader.cpp $(LNK_GL)

text2d.o: text2d.cpp text2d.h visibleObj.h
	g++ $(CFLAGS) -c text2d.cpp $(LNK_GL)

skeleton3d.o: skeleton3d.cpp skeleton3d.h visibleObj.h
	g++ $(CFLAGS) -c skeleton3d.cpp $(LNK_GL)

cloud.o: cloud.cpp cloud.h visibleObj.h
	g++ $(CFLAGS) -c cloud.cpp $(LNK_GL)

camera.o: camera.cpp camera.h
	g++ $(CFLAGS) -c camera.cpp $(LNK_GL)

visibleObj.o: visibleObj.cpp visibleObj.h
	g++ $(CFLAGS) -c visibleObj.cpp $(LNK_GL)

ransac.o: ransac.cpp ransac.hpp
	g++ $(CFLAGS) -fopenmp -c ransac.cpp

clean:
	rm -f *.o ViewPointCloud
	rm -f *.o PointCloudOrien
