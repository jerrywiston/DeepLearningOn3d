LNK_GL = -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -ldl -lXinerama -lXcursor
CFLAGS = -std=c++11 -O2

BoxUI: main.o text2d.o img2d.o skeleton3d.o cloud.o camera.o cursor.o obj3d.o visibleObj.o
	g++ $(CFLAGS) -o BoxUI main.o text2d.o img2d.o cursor.o obj3d.o skeleton3d.o cloud.o camera.o visibleObj.o $(LNK_GL)

main.o: main.cpp screen.hpp shader.hpp
	g++ $(CFLAGS) -c main.cpp $(LNK_GL)

text2d.o: text2d.cpp text2d.h visibleObj.h
	g++ $(CFLAGS) -c text2d.cpp $(LNK_GL)

img2d.o: img2d.cpp img2d.h visibleObj.h
	g++ $(CFLAGS) -c img2d.cpp $(LNK_GL)

cursor.o: cursor.cpp cursor.h obj3d.h visibleObj.h
	g++ $(CFLAGS) -c cursor.cpp $(LNK_GL)

obj3d.o: obj3d.cpp obj3d.h visibleObj.h
	g++ $(CFLAGS) -c obj3d.cpp $(LNK_GL)

skeleton3d.o: skeleton3d.cpp skeleton3d.h visibleObj.h
	g++ $(CFLAGS) -c skeleton3d.cpp $(LNK_GL)

cloud.o: cloud.cpp cloud.h visibleObj.h
	g++ $(CFLAGS) -c cloud.cpp $(LNK_GL)

camera.o: camera.cpp camera.h
	g++ $(CFLAGS) -c camera.cpp $(LNK_GL)

visibleObj.o: visibleObj.cpp visibleObj.h
	g++ $(CFLAGS) -c visibleObj.cpp $(LNK_GL)

clean:
	rm -f *.o BoxUI
