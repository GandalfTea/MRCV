cmake_minimum_required(VERSION 3.10)
project(MRCV VERSION 0.01)

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories("/usr/include/python2.7")
include_directories(include)
#file(GLOB SOURCES "src/*.cc")
set( SOURCES src/Frame.cc src/Map.cc)

set( CMAKE_CXX_FLAGS "-g")
add_executable(MRCV test/frame_test.cc ${SOURCES})


target_link_libraries(MRCV pango_display)
target_link_libraries(MRCV python2.7)
target_link_libraries(MRCV opencv_core opencv_highgui opencv_videoio opencv_imgproc opencv_features2d opencv_flann opencv_calib3d)
target_link_libraries(MRCV SDL GL GLU glut GLEW)
