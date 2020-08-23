cmake_minimum_required (VERSION 2.8)

set (PROJECT cyclone)

project (${PROJECT})

include_directories ("${CMAKE_SOURCE_DIR}/include")

file(GLOB HEADERS
	${CMAKE_SOURCE_DIR}/include/cyclone/*.h
)

file(GLOB SOURCES
	*.cpp)

add_library (${PROJECT} STATIC ${HEADERS} ${SOURCES})

add_subdirectory(demos)