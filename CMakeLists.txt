cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp2)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

option(LEPP_BUILD_EXAMPLES "Build LEPP examples" FALSE)
option(LEPP_BUILD_DETECTOR "Build an obstacle detector" TRUE)

include_directories("src")

if(LEPP_BUILD_EXAMPLES)
    include(examples/Examples.txt)
endif()

if(LEPP_BUILD_DETECTOR)
    file(GLOB detector_src src/lepp2/detector/*.cc src/lepp2/legacy/*.cpp)
    add_executable(detector ${detector_src})
    target_link_libraries(detector ${PCL_LIBRARIES})
endif()
