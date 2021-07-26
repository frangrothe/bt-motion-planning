//
// Created by francesco on 19.07.21.
//

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "MQ2DGoal.h"

namespace query2d {


MQ2DGoal::MQ2DGoal(const ompl::base::SpaceInformationPtr &si, double x, double y) : GoalSampleableRegion(si), x_(x),
                                                                                    y_(y) {}

double MQ2DGoal::distanceGoal(const ompl::base::State *st) const {
    double deltaX = fabs(x_- st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0]);
    double deltaY = fabs(y_- st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1]);

    return sqrt(deltaX * deltaX + deltaY * deltaY);
}

void MQ2DGoal::sampleGoal(ompl::base::State *st) const {
    auto x = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    auto y = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];

    st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x_;
    st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y_;
}

unsigned int MQ2DGoal::maxSampleCount() const {
    return 1;
}
}