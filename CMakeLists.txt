cmake_minimum_required(VERSION 2.8.12)
project(polar_grid_image_cluster)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(OpenCV REQUIRED)

add_definitions( ${PCL_DEFINITIONS} )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} )

add_executable(polar_grid_image_cluster polar_grid_image_cluster.cpp)
target_link_libraries(polar_grid_image_cluster ${EIGEN3_INCLUDE_DIR} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIRS} ${OpenCV_LIBS})

