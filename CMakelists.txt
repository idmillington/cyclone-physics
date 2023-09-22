cmake_minimum_required(VERSION 3.25.0)
project(CyclonePhysics)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT CycloneTest)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CXX_STANDARD_REQUIRED ON)

if (MSVC)
	# Ignore warnings about missing pdb
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ignore:4099")
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /ignore:4099")
	set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} /ignore:4099")
endif()

find_package(glm REQUIRED)
find_package(freeglut REQUIRED)
find_package(fmtlog REQUIRED)
find_package(OpenGL REQUIRED)
find_package(catch2 REQUIRED)

# Physics library
add_subdirectory(Lib)
# Unit tests
add_subdirectory(Test)
# Demos
add_subdirectory(Demo)