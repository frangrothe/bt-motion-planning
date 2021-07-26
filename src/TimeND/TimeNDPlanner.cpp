//
// Created by francesco on 25.07.21.
//


#include "TimeNDPlanner.h"



namespace nd {


void TimeNDPlanner::planMotion() {

    double distance = sqrt(d_); // distance between start (0, ..., 0) and goal (1, ..., 1)

    auto vectorSpace = std::make_shared<ob::RealVectorStateSpace>(d_);
    auto space = std::make_shared<space_time::AnimationStateSpace>(vectorSpace, distance, timeWeight_);

    // Set the bounds for R1
    ob::RealVectorBounds bounds(d_);
    bounds.setLow(0.0);
    bounds.setHigh(1.01);
    vectorSpace->setBounds(bounds);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<TimeNDStateValidityChecker>(si, d_, constraintTime_));
    si->setMotionValidator(std::make_shared<TimeNDMotionValidator>(si, d_, distance));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    ob::ScopedState<> start(space);
    start = 0.0; // set real vector value in each dimension
    ob::ScopedState<> goal(space);
    goal = 1.0; // set real vector value in each dimension
    auto goalRegion = std::make_shared<TimeNDGoal>(si, d_);
    goalRegion->setState(goal);

    pdef->addStartState(start);
    pdef->setGoal(goalRegion);

    //(3) Planner
    auto planner(std::make_shared<space_time::SpaceTimeRRT>(si));
    planner->setRange(plannerRangeFactor_ * distance);
    planner->setRewiringToKNearest();
    planner->setProblemDefinition(pdef);
    planner->setBatchSize(batchSize_);
    planner->setup();

    //(4) Planner executes
    ob::PlannerStatus solved = planner->ob::Planner::solve(solveTime_);
    if (solved)
    {
        ob::PlannerData data(si);
        planner->getPlannerData(data);

        std::cout << "\nFound solution:" << std::endl;

        // print the path to screen
        ob::PathPtr path = pdef->getSolutionPath();
        path->print(std::cout);

        std::cout << "\nNumber of Samples: " << data.numVertices();
        std::cout << "\nExact Solution: " << (pdef->hasExactSolution()? "yes" : "no");
        std::cout << "\nApproximate Solution: " << (pdef->hasApproximateSolution()? "yes" : "no") << std::endl;

    }
    else
        std::cout << "No solution found" << std::endl;
}
}