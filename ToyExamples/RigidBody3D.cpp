//
// Created by francesco on 12.05.21.
//


#include "RigidBody3D.h"

namespace rb3d {

bool isStateValid(const ob::State *state) {
    return true;
}

void planRigidBody3D() {
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set bounds of R3 for this state space
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    //create Simple Setup
    og::SimpleSetup ss(space);

    // set state validity checker
    ss.setStateValidityChecker([] (const ob::State *state) {
        return isStateValid(state);
    });

    // Create start and goal states
    ob::ScopedState<> start(space);
    start.random();
    ob::ScopedState<> goal(space);
    goal.random();

    // Set created states as start and goal for SimpleSetup
    ss.setStartAndGoalStates(start, goal);

    // Try to solve the problem
    ob::PlannerStatus solved = ss.solve(1.0);

    // Display Solution
    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
}
}




