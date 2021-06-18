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

include_directories(Time1D)


add_executable(bt_robotics
        main.cpp
        auxillary.cpp auxillary.h
        Time1D/Time1DPlanner.cpp Time1D/Time1DPlanner.h
        Time1D/Time1DStateValidityChecker.cpp Time1D/Time1DStateValidityChecker.h
        Time1D/Time1DMotionValidator.cpp Time1D/Time1DMotionValidator.h
        Time1D/Time1DGoal.cpp Time1D/Time1DGoal.h
        Time1D/Time1DGoalRegion.cpp Time1D/Time1DGoalRegion.h
        Time1D/Time1DValidStateSampler.cpp Time1D/Time1DValidStateSampler.h
        Time2D/SpaceTimePlanner.cpp Time2D/SpaceTimePlanner.h
        SpaceTimePlanning/AnimationStateSpace.cpp SpaceTimePlanning/AnimationStateSpace.h
        Time2D/SpaceTimeStateValidityChecker.cpp Time2D/SpaceTimeStateValidityChecker.h
        Time2D/SpaceTimeMotionValidator.cpp Time2D/SpaceTimeMotionValidator.h
        Time1D/Constraint.h
        structs/Constraint2D.h
        Time2D/goals/SpaceTimeSampler.cpp Time2D/goals/SpaceTimeSampler.h
        Time2D/goals/SpaceTimeGoalRegion.cpp Time2D/goals/SpaceTimeGoalRegion.h
        json.h
        SpaceTimePlanning/SpaceTimeRRT.cpp SpaceTimePlanning/SpaceTimeRRT.h)

