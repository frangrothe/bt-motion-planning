//
// Created by francesco on 27.05.21.
//

#include "Time2DStateValidityChecker.h"

#include <utility>

namespace time_2d {

Time2DStateValidityChecker::Time2DStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                       ob::RealVectorBounds bounds,
                                                       std::vector<Constraint> constraints, double agentWidth,
                                                       double agentHeight) : StateValidityChecker(si), bounds_(std::move(bounds)),
                                                       constraints_(std::move(constraints)),
                                                       agentWidth_(agentWidth),
                                                       agentHeight_(agentHeight) {}

bool Time2DStateValidityChecker::isValid(const ompl::base::State *state) const {
    double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    double minX = x - agentWidth_ / 2;
    double maxX = x + agentWidth_ / 2;
    double minY = y - agentHeight_ / 2;
    double maxY = y + agentHeight_ / 2;

    if (!isInBounds(minX, maxX, minY, maxY)) return false;

    return std::none_of(constraints_.cbegin(), constraints_.cend(), [minX, maxX, minY, maxY, t] (const Constraint& c) {
        return c.checkAABBCollision(minX, maxX, minY, maxY, t);
    });

}

bool Time2DStateValidityChecker::isInBounds(double minX, double maxX, double minY, double maxY) const {
    return minX >= bounds_.low[0] && maxX <= bounds_.high[0] && minY >= bounds_.low[1] && maxY <= bounds_.high[1];
}
}