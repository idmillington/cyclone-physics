project(CycloneTest)

file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS 
	${CMAKE_CURRENT_SOURCE_DIR}/source/*.h 
	${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES})

add_executable (${PROJECT_NAME} ${SOURCE_FILES})
add_test (${PROJECT_NAME} ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
	CXX_STANDARD 17
)

target_link_libraries (${PROJECT_NAME}
	PRIVATE CycloneLib
	PRIVATE Catch2::Catch2WithMain
)

target_include_directories(${PROJECT_NAME}
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/source/
)