# project
cmake_minimum_required (VERSION 3.1.0)
project (libelas)

set(CMAKE_CXX_FLAGS "-msse3")
set(CMAKE_BUILD_TYPE Release)

set (LIBELAS_SRC_DIR src)
include_directories(${LIBELAS_SRC_DIR})
FILE(GLOB LIBELAS_SRC_FILES "src/*.cpp")

set(HDRS_LIBELAS
	src/descriptor.h
	src/elas.h
	src/filter.h
	src/image.h
	src/matrix.h
	src/timer.h
	src/triangle.h
)

set(SRCS_LIBELAS
	src/descriptor.cpp
	src/elas.cpp
	src/filter.cpp
	src/matrix.cpp
	src/triangle.cpp
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(ELAS SHARED ${SRCS_LIBELAS} ${HDRS_LIBELAS})
