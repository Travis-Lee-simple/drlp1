cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(AIRobot)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (AiRobot_Cam2 main.cpp)
target_link_libraries (AiRobot_Cam2 ${PCL_LIBRARIES})
