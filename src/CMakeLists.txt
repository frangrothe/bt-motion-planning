cmake_minimum_required(VERSION 3.19)
project(bt_robotics)
set(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(ompl 1.5.0 REQUIRED)
MESSAGE(${OMPL_VERSION})
MESSAGE(${OMPL_INCLUDE_DIRS})
MESSAGE(${OMPL_LIBRARIES})

list(APPEND INCLUDE_DIRS ${OMPL_INCLUDE_DIRS})
list(APPEND LIBRARIES ${OMPL_LIBRARIES})

include_directories(${INCLUDE_DIRS})
link_libraries(${LIBRARIES})

include_directories(ToyExamples)


add_executable(bt_robotics main.cpp ToyExamples/RigidBody3D.h ToyExamples/RigidBody3D.cpp ToyExamples/Time1D.cpp ToyExamples/Time1D.h auxillary.cpp auxillary.h)

