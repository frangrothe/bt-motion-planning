//
// Created by francesco on 27.05.21.
//

#include "SpaceTimePlanner.h"

namespace SpaceTime {

SpaceTimePlanner::SpaceTimePlanner(std::string filename) : filename_(std::move(filename)) {}

void SpaceTimePlanner::planMotion() {

    //(1) Define State Space
    auto space = std::make_shared<AnimationStateSpace>(1);
    // Set the bounds
    ob::RealVectorBounds bounds(1);
    bounds.setLow(xBoundLow_);
    bounds.setHigh(xBoundHigh_);
    space->setVectorBounds(bounds);
    space->setTimeBounds(timeBoundLow_, timeBoundHigh_);
//
//    auto r1State(std::make_shared<ob::RealVectorStateSpace>(1));
//    auto timeState(std::make_shared<ob::TimeStateSpace>());
//    auto space = r1State + timeState;
//
//    // Set the bounds for R1
//    ob::RealVectorBounds bounds(1);
//    bounds.setLow(xBoundLow_);
//    bounds.setHigh(xBoundHigh_);
//    r1State->setBounds(bounds);
//    timeState->setBounds(timeBoundLow_, timeBoundHigh_);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<SpaceTimeStateValidityChecker>(si, constraints_));
    si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si, vMax_));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    ob::ScopedState<> start(space);
    start[0] = xStart_; // r1State

    minTime_ = (xGoalRegionLeft_ - xStart_) / vMax_; // minimum time at which the goal can be reached
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<SpaceTimeGoalRegion>(si, xGoalRegionLeft_, xGoalRegionRight_, minTime_, timeBoundHigh_));

    //(3) Planner
    auto planner(std::make_shared<og::RRTConnect>(si));
    planner->setRange(plannerRange_);
    planner->setProblemDefinition(pdef);
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

        auxillary::writeSamplesToCSV(data, filename_);
        auxillary::writePathToCSV(path, filename_);
        auxillary::writeConstraintsToCSV(constraints_, filename_);
        auxillary::writeGoalRegionToCSV(xGoalRegionLeft_, xGoalRegionRight_, minTime_, timeBoundHigh_, filename_);
    }
    else
        std::cout << "No solution found" << std::endl;

}
}