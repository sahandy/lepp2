cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp2)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

option(LEPP_BUILD_EXAMPLES "Build LEPP examples" TRUE)
option(LEPP_BUILD_DETECTOR "Build an obstacle detector" TRUE)
option(LEPP_BUILD_LOLA "Build an obstacle detector for LOLA" TRUE)

include_directories("../src")

if(LEPP_BUILD_EXAMPLES)
    include(../examples/Examples.txt)
endif()

if(LEPP_BUILD_DETECTOR)
    file(GLOB detector_src lepp2/detector/*.cc)
    add_executable(detector ${detector_src})
    target_link_libraries(detector ${PCL_LIBRARIES})
endif()

if(LEPP_BUILD_LOLA)
    file(GLOB lola_src lola/*.cc)
    add_executable(lola ${lola_src})
    target_link_libraries(lola ${PCL_LIBRARIES})
endif()
