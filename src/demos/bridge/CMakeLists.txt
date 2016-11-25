set (PROJECT bridge)

project (${PROJECT})

include_directories(${COMMON_DEMO_INCLUDE})

file(GLOB HEADERS
	*.h)

file(GLOB SOURCES
	*.cpp)

SOURCE_GROUP("common/include" FILES ${COMMON_DEMO_HEADERS})
SOURCE_GROUP("common/src" FILES ${COMMON_DEMO_SOURCE})

SOURCE_GROUP("include" FILES ${HEADERS})
SOURCE_GROUP("src" FILES ${SOURCES})

add_executable(${PROJECT} ${HEADERS} ${SOURCES} 
	${COMMON_DEMO_HEADERS} ${COMMON_DEMO_SOURCE} )

target_link_libraries(${PROJECT} cyclone ${COMMON_DEMO_LIBS})
