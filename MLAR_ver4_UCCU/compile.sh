g++ -std=c++11 -O2 -c visibleObj.cpp -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor

g++ -std=c++11 -O2 -c camera.cpp -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor

g++ -std=c++11 -O2 -c cloud.cpp -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor

g++ -std=c++11 -O2 -c skeleton3d.cpp -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor

g++ -std=c++11 -O2 -c text2d.cpp -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor

g++ -std=c++11 -O2 -c shader.cpp -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor

g++ -std=c++11 -O2 -fopenmp -c ICP_fusion.cpp

nvcc -std=c++11 -O2 -c voxelGrid.cu --gpu-architecture=compute_50 --gpu-code=compute_50

g++ -std=c++11 -O2 -c main.cpp -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor

nvcc -Xcompiler -fopenmp -std=c++11 -O2 -o MLAR main.o voxelGrid.o ICP_fusion.o shader.o text2d.o skeleton3d.o cloud.o camera.o visibleObj.o -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor -lgomp --gpu-architecture=compute_50 --gpu-code=compute_50
