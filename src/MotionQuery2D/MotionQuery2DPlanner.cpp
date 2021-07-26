//
// Created by francesco on 19.07.21.
//

#include "MotionQuery2DPlanner.h"

namespace query2d {


MotionQuery2DPlanner::MotionQuery2DPlanner(double xStart, double yStart, double xGoal, double yGoal) : xStart_(xStart),
                                                                                                       yStart_(yStart),
                                                                                                       xGoal_(xGoal),
                                                                                                       yGoal_(yGoal) {}

void MotionQuery2DPlanner::planMotion() {

    //(1) Define State Space
    auto vectorSpace = std::make_shared<ob::RealVectorStateSpace>(2);
    auto stateSpace = std::make_shared<space_time::AnimationStateSpace>(vectorSpace, vMax_, timeWeight_);

    // Set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, xBoundLow_);
    bounds.setHigh(0, xBoundHigh_);
    bounds.setLow(1, yBoundLow_);
    bounds.setHigh(1, yBoundHigh_);

    vectorSpace->setBounds(bounds);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(stateSpace);
    si->setStateValidityChecker(std::make_shared<MQ2DStateValidityChecker>(si, staticConstraints_, pathConstraints_, bounds, radius_));
    si->setMotionValidator(std::make_shared<MQ2DMotionValidator>(si, 2, vMax_));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    ob::ScopedState<> start(stateSpace);
    start[0] = xStart_; // r1State
    start[1] = yStart_;

    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<MQ2DGoal>(si, xGoal_, yGoal_));

    //(3) Planner
    auto planner(std::make_shared<space_time::SpaceTimeRRT>(si));
    planner->setRange(plannerRange_);
    planner->setRewiringToKNearest();
    planner->setBatchSize(1000);
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
        solutionPath_ = path;

        std::cout << "\nNumber of Samples: " << data.numVertices();
        std::cout << "\nExact Solution: " << (pdef->hasExactSolution()? "yes" : "no");
        std::cout << "\nApproximate Solution: " << (pdef->hasApproximateSolution()? "yes" : "no") << std::endl;

    }
    else
        std::cout << "No solution found" << std::endl;
}

void MotionQuery2DPlanner::writeConstraintsToJSON(const std::string& filename) {
    using json = nlohmann::json;
    json j;

    std::vector<std::vector<double>> jsonVector{};
    for (auto &c : staticConstraints_) {
        jsonVector.push_back({c.x, c.y, c.radius});
    }

    j = jsonVector;
    // write prettified JSON to another file
    std::ofstream outfile("data/" + filename + "/constraints.json");
    outfile << std::setw(4) << j << std::endl;

}

void MotionQuery2DPlanner::writeSolutionToJSON(const std::string& filename, int index) {
    using json = nlohmann::json;
    json j;

    std::vector<std::vector<double>> jsonVector{};

    auto path = solutionPath_->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        jsonVector.push_back({x, y, t});
    }

    j = jsonVector;
    // write prettified JSON to another file
    std::ofstream outfile("data/" + filename + "/solution" + std::to_string(index) + ".json");
    outfile << std::setw(4) << j << std::endl;
}

std::vector<Point> MotionQuery2DPlanner::getPathConstraint() {
    std::vector<Point> v{};
    auto path = solutionPath_->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        v.emplace_back(x, y, t);
    }

    return v;
}
}