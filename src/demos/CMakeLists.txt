cmake_minimum_required (VERSION 2.8)

find_package(OpenGL REQUIRED)

if(MSVC)
	find_package(GLUT QUIET)
	if(NOT GLUT_FOUND)
		set(GLUT_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/contrib/freeglut/include")
		if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
			set(GLUT_LIBRARIES "${CMAKE_SOURCE_DIR}/contrib/freeglut/lib/x64/freeglut.lib")
			file(COPY "${CMAKE_SOURCE_DIR}/contrib/freeglut/bin/x64/freeglut.dll"
		    	DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
		else()
			set(GLUT_LIBRARIES "${CMAKE_SOURCE_DIR}/contrib/freeglut/lib/freeglut.lib")
			file(COPY "${CMAKE_SOURCE_DIR}/contrib/freeglut/bin/freeglut.dll"
		    	DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
		endif()
	endif()
else()
	find_package(GLUT REQUIRED)
endif()

if(WIN32)
    set(PLATFORM_LIBRARIES winmm.lib)
endif()

file(GLOB COMMON_DEMO_HEADERS
	${CMAKE_SOURCE_DIR}/src/demos/*.h
)

file(GLOB COMMON_DEMO_SOURCE
	${CMAKE_SOURCE_DIR}/src/demos/*.cpp
)

set(COMMON_DEMO_INCLUDE "${GLUT_INCLUDE_DIR}" "${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/src/demos")
set(COMMON_DEMO_LIBS ${GLUT_glut_LIBRARY} ${OPENGL_LIBRARIES} ${PLATFORM_LIBRARIES})

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SAFESEH:NO")
endif()

add_subdirectory(ballistic)
add_subdirectory(bigballistic)
add_subdirectory(blob)
add_subdirectory(bridge)
add_subdirectory(explosion)
add_subdirectory(fireworks)
add_subdirectory(flightsim)
add_subdirectory(fracture)
add_subdirectory(platform)
add_subdirectory(ragdoll)
add_subdirectory(sailboat)