CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin
	LDFLAGS = -lglut -lGLU
	FLAGS += -O3
	FLAGS += -std=c++11
	FLAGS += -D_DEBUG -g Wall
endif
	
all: main 
main: scene.o 
	$(CC) $(CFLAGS) -o as4 scene.o $(LDFLAGS) 
scene.o: scene.cpp
	$(CC) $(CFLAGS) -c scene.cpp -o scene.o
clean: 
	$(RM) *.o as4
 


