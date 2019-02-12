cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++11)

project(kalman_tracking)

find_package(OpenCV 2.4.13 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    inc
)

add_executable(
    tracker

    main.cpp
    src/object.cpp
    src/objectsimulator.cpp
    src/visualizer.cpp
    src/tracker.cpp
)

target_link_libraries(
    tracker
    ${OpenCV_LIBS}
)
