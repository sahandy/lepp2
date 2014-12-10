cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp2)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

option(LEPP_BUILD_EXAMPLES "Build LEPP examples" FALSE)

include_directories("src")

if(LEPP_BUILD_EXAMPLES)
  include(examples/Examples.txt)
endif()
