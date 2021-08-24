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
        Time1D/Time1DGoalRegion.cpp Time1D/Time1DGoalRegion.h
        Time1D/Time1DValidStateSampler.cpp Time1D/Time1DValidStateSampler.h
        Time2D/Time2DPlanner.cpp Time2D/Time2DPlanner.h
        SpaceTimePlanning/AnimationStateSpace.cpp SpaceTimePlanning/AnimationStateSpace.h
        Time2D/Time2DStateValidityChecker.cpp Time2D/Time2DStateValidityChecker.h
        Time2D/Time2DMotionValidator.cpp Time2D/Time2DMotionValidator.h
        Time1D/Constraint.h
        Time2D/Constraint.h
        Time2D/goals/Time2DSampler.cpp Time2D/goals/Time2DSampler.h
        Time2D/goals/Time2DGoalRegion.cpp Time2D/goals/Time2DGoalRegion.h
        json.h
        SpaceTimePlanning/SpaceTimeRRT.cpp SpaceTimePlanning/SpaceTimeRRT.h
        Dubins/DubinsPlanner.cpp Dubins/DubinsPlanner.h
        Dubins/DubinsStateValidityChecker.cpp Dubins/DubinsStateValidityChecker.h
        Dubins/DubinsMotionValidator.cpp Dubins/DubinsMotionValidator.h
        Dubins/goals/DubinsGoalRegion.cpp Dubins/goals/DubinsGoalRegion.h
        Dubins/goals/DubinsGoalSampler.cpp Dubins/goals/DubinsGoalSampler.h
        Dubins/DubinsConstraint.h Time1D/goals/VectorSpaceGoalRegion.cpp Time1D/goals/VectorSpaceGoalRegion.h MotionQuery2D/MotionQuery2DPlanner.cpp MotionQuery2D/MotionQuery2DPlanner.h MotionQuery2D/StaticConstraint.h MotionQuery2D/MQ2DStateValidityChecker.cpp MotionQuery2D/MQ2DStateValidityChecker.h MotionQuery2D/MQ2DMotionValidator.cpp MotionQuery2D/MQ2DMotionValidator.h MotionQuery2D/MQ2DGoal.cpp MotionQuery2D/MQ2DGoal.h MotionQuery2D/QueryExecutor.cpp MotionQuery2D/QueryExecutor.h MotionQuery2D/Point.h TimeND/TimeNDPlanner.cpp TimeND/TimeNDPlanner.h TimeND/TimeNDStateValidityChecker.cpp TimeND/TimeNDStateValidityChecker.h TimeND/TimeNDMotionValidator.cpp TimeND/TimeNDMotionValidator.h TimeND/TimeNDGoal.cpp TimeND/TimeNDGoal.h TimeND/TimeNDConstraint.h Benchmark/Benchmarker.cpp Benchmark/Benchmarker.h SpaceTimePlanning/MinimizeArrivalTime.cpp SpaceTimePlanning/MinimizeArrivalTime.h)

