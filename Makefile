

# LINK FLAGS AND PLATFORM

# Linux (default)
LDFLAGS = -lGL -lGLU -lglut

# OS X
ARCH = $(shell uname)
ifeq ($(ARCH),Darwin)
        LDFLAGS = -framework GLUT -framework OpenGL -framework Cocoa
endif

mkdir=mkdir -p
rm=rm -f
AR=ar cq
RANLIB=ranlib


# CYCLONEPHYSICS LIB
CXXFLAGS=-O2 -I./include -fPIC
CYCLONEOBJS=src/body.o src/collide_coarse.o src/collide_fine.o src/contacts.o src/core.o src/fgen.o src/joints.o src/particle.o src/pcontacts.o src/pfgen.o src/plinks.o src/pworld.o src/random.o src/world.o


# DEMO FILES

LIBNAME=libcyclone.a
CYCLONELIB=./lib/linux/$(LIBNAME)

DEMO_CPP=./src/demos/app.cpp ./src/demos/timing.cpp ./src/demos/main.cpp

DEMOS=ballistic bigballistic blob bridge explosion fireworks flightsim fracture platform ragdoll sailboat


# OUTPUT DIRECTORIES

OUTDIRS=./lib/linux ./bin/linux



# BUILD COMMANDS

all:	out_dirs $(CYCLONELIB) $(DEMOS)


out_dirs: 
	$(mkdir) $(OUTDIRS)


$(CYCLONELIB): $(CYCLONEOBJS)
	$(rm) $@
	$(AR) $@ $(CYCLONEOBJS)
	$(RANLIB) $@


$(DEMOS): 
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o ./bin/linux/$@ $(DEMO_CPP) $(CYCLONELIB) ./src/demos/$@/$@.cpp


clean:
	$(rm) src/*.o lib/linux/libcyclone.a
	$(rm)		\
	./bin/linux/fireworks		\
	./bin/linux/fracture		\
	./bin/linux/flightsim		\
	./bin/linux/bridge		\
	./bin/linux/sailboat		\
	./bin/linux/explosion		\
	./bin/linux/ballistic		\
	./bin/linux/platform		\
	./bin/linux/bigballistic	\
	./bin/linux/blob		\
	./bin/linux/ragdoll
