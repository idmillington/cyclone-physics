# CYCLONEPHYSICS LIB
CXXFLAGS=-O2 -I./include -fPIC
CYCLONEOBJS=src/body.o src/collide_coarse.o src/collide_fine.o src/contacts.o src/core.o src/fgen.o src/joints.o src/particle.o src/pcontacts.o src/pfgen.o src/plinks.o src/pworld.o src/random.o src/world.o


# DEMO FILES
SOURCES=body
LDFLAGS=-lGL -lglut -lGLU -L./Debug -std=c++0x
DEMO_CPP=./src/demos/app.cpp ./src/demos/timing.cpp ./src/demos/main.cpp
CYCLONELIB=./lib/libcyclone.so

DEMOS=ballistic bigballistic blob bridge explosion fireworks flightsim fracture platform ragdoll sailboat



all:	libcyclone.so $(DEMOS)

libcyclone.so: $(CYCLONEOBJS)
	$(CXX) $(CYCLONEOBJS) -shared -dynamiclib -o lib/libcyclone.so

$(DEMOS): 
	$(CXX) -o ./bin/$@ $(DEMO_CPP) $(CYCLONELIB) ./src/demos/$@/$@.cpp $(CXXFLAGS) $(LDFLAGS) 


clean:
	rm -f src/*.o lib/libcyclone.so
	rm -f 		\
	./bin/fireworks		\
	./bin/fracture		\
	./bin/flightsim		\
	./bin/bridge		\
	./bin/sailboat		\
	./bin/explosion		\
	./bin/ballistic		\
	./bin/platform		\
	./bin/bigballistic	\
	./bin/blob		\
	./bin/ragdoll
