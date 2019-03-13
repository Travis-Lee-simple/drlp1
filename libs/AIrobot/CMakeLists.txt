cmake_minimum_required(VERSION 2.8.7)
project(sample)


find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O3 -march=core2")

find_package(OpenCV 2 REQUIRED)

set(ROOT 		"${CMAKE_CURRENT_SOURCE_DIR}/")
set(LIB_REALSENSE 	"/usr/lib/x86_64-linux-gnu/librealsense2.so")
set(LIB_ROBOT "/home/chen/AIrobot/robotSDK/lib/linux_x64/libauborobotcontroller.so.1")
set(LIB_REALSENSE_INC "/usr/include")

include_directories(
	${LIB_REALSENSE_INC}
    ${ROOT}/robotSDK/inc
    ${OpenCV_INCLUDE_DIRS}
    ${ROOT}
)

link_directories(
    ${ROOT}/robotSDK/lib/linux_x64
)

file(GLOB SOURCES
    "*.cpp"
    )

set( PROJECT_LINK_LIBS
    ${LIB_REALSENSE}
    ${LIB_ROBOT}
    ${OpenCV_LIBRARIES}
)
add_executable(sample ${SOURCES})
target_link_libraries(sample ${PROJECT_LINK_LIBS})

target_link_libraries(sample ${PCL_LIBRARIES})



