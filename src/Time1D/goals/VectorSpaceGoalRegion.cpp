//
// Created by francesco on 04.07.21.
//

#include "VectorSpaceGoalRegion.h"

#include <utility>

namespace time_1d {


VectorSpaceGoalRegion::VectorSpaceGoalRegion(const ompl::base::SpaceInformationPtr &si, const std::vector<std::pair<double, double>> &goals)
        : GoalSampleableRegion(si), goals_(goals), sampler_(&(*si), goals) {}

double VectorSpaceGoalRegion::distanceGoal(const ompl::base::State *st) const {
    double x = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double minDist = std::numeric_limits<double>::infinity();
    for (auto &g : goals_) {
        double dist = x < g.first ? g.first - x : x > g.second ? x - g.second : 0.0;
        if (dist < minDist)
            minDist = dist;
    }
    return minDist;
}

void VectorSpaceGoalRegion::sampleGoal(ompl::base::State *st) const {

    st->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = 0;
    sampler_.sample(st);
}

unsigned int VectorSpaceGoalRegion::maxSampleCount() const {
    bool regions = false;
    for (auto &g : goals_) {
        if (g.first != g.second) {
            regions = true;
            break;
        }
    }
    if (regions)
        return std::numeric_limits<unsigned int>::max();
    else
        return goals_.size();
}
}
