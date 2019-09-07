cmake_minimum_required(VERSION 3.0) # minimum requirement for Hunter

project(arm1632)

set(BOOST_THREAD_LIBRARY boost_thread -pthread)
add_definitions(-std=c++14 -lboost_system)
#find_package(
#        PCL REQUIRED
#)
find_package(Threads REQUIRED)
find_package(
        OpenCV REQUIRED
)

include_directories (
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(
        ${OpenCV_LIB_DIR}
        #${PCL_LIBRARY_DIRS}
        /home/xcy/arm1632/hitbot_lib
)

add_executable(armTest main.cpp)
target_link_libraries(armTest small_scara_interface pthread)

