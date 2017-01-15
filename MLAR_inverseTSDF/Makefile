LNK_GL = -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor
CUDA_OPT = --gpu-architecture=compute_50 --gpu-code=compute_50
CFLAGS = -std=c++11 -O2

MLAR: main.o voxelGrid.o ICP_fusion.o shader.o text2d.o img2d.o skeleton3d.o cloud.o camera.o visibleObj.o kdtree.o Eigen_kdtree.o
	nvcc -Xcompiler -fopenmp $(CFLAGS) -o MLAR main.o voxelGrid.o ICP_fusion.o shader.o text2d.o img2d.o skeleton3d.o cloud.o camera.o visibleObj.o kdtree.o Eigen_kdtree.o -lgomp $(LNK_GL) $(CUDA_OPT)

main.o: main.cpp voxelGrid.h config.hpp
	g++ $(CFLAGS) -c main.cpp $(LNK_GL)

voxelGrid.o: voxelGrid.cu ICP_fusion.h voxelGrid.h skeleton3d.h cloud.h camera.h shader.hpp config.hpp
	nvcc $(CFLAGS) -c voxelGrid.cu $(CUDA_OPT)

kdtree.o: kdtree.c kdtree.h
	gcc -O2 -c kdtree.c

Eigen_kdtree.o: Eigen_kdtree.cpp Eigen_kdtree.hpp kdtree.h
	g++ $(CFLAGS) -c Eigen_kdtree.cpp

ICP_fusion.o: ICP_fusion.cpp ICP_fusion.h voxelGrid.h ICP.h ICP_kdtree.hpp Eigen_kdtree.hpp
	g++ $(CFLAGS) -fopenmp -c ICP_fusion.cpp

shader.o: shader.cpp shader.hpp
	g++ $(CFLAGS) -c shader.cpp $(LNK_GL)

text2d.o: text2d.cpp text2d.h visibleObj.h
	g++ $(CFLAGS) -c text2d.cpp $(LNK_GL)

img2d.o: img2d.cpp img2d.h visibleObj.h
	g++ $(CFLAGS) -c img2d.cpp $(LNK_GL)

skeleton3d.o: skeleton3d.cpp skeleton3d.h visibleObj.h
	g++ $(CFLAGS) -c skeleton3d.cpp $(LNK_GL)

cloud.o: cloud.cpp cloud.h visibleObj.h
	g++ $(CFLAGS) -c cloud.cpp $(LNK_GL)

camera.o: camera.cpp camera.h
	g++ $(CFLAGS) -c camera.cpp $(LNK_GL)

visibleObj.o: visibleObj.cpp visibleObj.h
	g++ $(CFLAGS) -c visibleObj.cpp $(LNK_GL)

clean:
	rm *.o MLAR
