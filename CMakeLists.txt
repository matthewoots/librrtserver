cmake_minimum_required(VERSION 2.8.3)
project(librrtserver)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

file(GLOB INCLUDE_FILES "include/*.h")
file(GLOB SRC_FILES "src/*.cpp")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common filters kdtree)
include_directories(
    ${EIGEN3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    include
)

add_executable(${PROJECT_NAME}_node 
    src/rrt_server_module.cpp
    src/test_server_module.cpp
)
target_link_libraries(${PROJECT_NAME}_node 
    ${PCL_LIBRARIES})

include(CTest)

add_library(${PROJECT_NAME} STATIC ${SRC_FILES} ${INCLUDE_FILES})
target_include_directories (${PROJECT_NAME} PUBLIC 
    ${PROJECT_SOURCE_DIR} 
    ${PROJECT_SOURCE_DIR}/include)
