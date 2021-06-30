//
// Created by francesco on 21.06.21.
//


#include "DubinsPlanner.h"

namespace dubins {

DubinsPlanner::DubinsPlanner(std::string filename) : filename_(std::move(filename)) {}

void DubinsPlanner::planMotion() {
    //(1) Define State Space
    auto dubinsSpace = std::make_shared<ob::DubinsStateSpace>();
    auto stateSpace = std::make_shared<space_time::AnimationStateSpace>(dubinsSpace);

    // Set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, xBoundLow_);
    bounds.setHigh(0, xBoundHigh_);
    bounds.setLow(1, yBoundLow_);
    bounds.setHigh(1, yBoundHigh_);

//    dubinsSpace->setBounds(bounds);
    stateSpace->setTimeBounds(timeBoundLow_, timeBoundHigh_);
    stateSpace->getSpaceComponent()->as<ob::SE2StateSpace>()->setBounds(bounds);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(stateSpace);
    si->setStateValidityChecker(std::make_shared<DubinsStateValidityChecker>(si, constraints_));
    si->setMotionValidator(std::make_shared<DubinsMotionValidator>(si, vMax_));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    ob::ScopedState<> start(stateSpace);
    start[0] = xStart_; // r1State
    start[1] = yStart_;
    start[2] = yawStart_;

    // lazy min time calculation
    minTime_ = (xGoal_ - xStart_) / vMax_;
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<DubinsGoalRegion>(si, xGoal_, yGoal_, yawGoal_, minTime_, timeBoundHigh_));

    //(3) Planner
    auto planner(std::make_shared<og::RRTConnect>(si));
//    auto planner(std::make_shared<space_time::SpaceTimeRRT>(si));
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

//        writeSamplesToCSV(data);
        writePathToCSV(path);
//        writeConstraintsToJSON();
//        writeGoalRegionToCSV();
    }
    else
        std::cout << "No solution found" << std::endl;

}

void DubinsPlanner::writePathToCSV(const ompl::base::PathPtr &pathPtr) {
    std::ofstream outfile ("data/" + filename_ + "/path.csv");

    outfile << "x" << delim_ << "y" << delim_ << "yaw" << delim_ << "time\n";

    auto path = pathPtr->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getX();
        double y = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getY();
        double yaw = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getYaw();
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        outfile << x << delim_ << y << delim_ << yaw << delim_ << t << "\n";
    }

    outfile.close();
}
}