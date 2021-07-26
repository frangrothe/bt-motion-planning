//
// Created by francesco on 19.07.21.
//

#include "MQ2DStateValidityChecker.h"

namespace query2d {

MQ2DStateValidityChecker::MQ2DStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                   std::vector<StaticConstraint> staticConstraints,
                                                   std::vector<std::vector<Point>> pathConstraints,
                                                   ob::RealVectorBounds bounds, double agentRadius)
        : StateValidityChecker(si), staticConstraints_(std::move(staticConstraints)), pathConstraints_(std::move(pathConstraints)),
          bounds_(std::move(bounds)), agentRadius_(agentRadius) {}

bool MQ2DStateValidityChecker::isValid(const ompl::base::State *state) const {
    double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    if (!isInBounds(x, y))
        return false;

    // check static obstacles
    for (auto &c : staticConstraints_) {
        double dist = sqrt((x - c.x) * (x - c.x) + (y - c.y) * (y - c.y));
        if (dist < agentRadius_ + c.radius)
            return false;
    }

    // check moving obstacles
    for (auto &c : pathConstraints_) {
        int iLeft, iRight;
        bool inTime = false;
        // find left and right points for the specific time
        for (int i = 0; i < c.size(); ++i) {
            if (c[i].t >= t) {
                iLeft = i == 0 ? 0 : i - 1;
                iRight = iLeft + 1;
                inTime = true;
                break;
            }
        }

        double cx, cy;
        if (!inTime) {
            cx = c.back().x;
            cy = c.back().y;
        } else {
            auto &left = c[iLeft];
            auto &right = c[iRight];
            // interpolate obstacles position
            double frac = (t - left.t) / (right.t - left.t);
            cx = left.x + frac * (right.x - left.x);
            cy = left.y + frac * (right.y - left.y);
        }

        double dist = sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
        if (dist < agentRadius_ + agentRadius_)
            return false;
    }

    return true;
}

bool MQ2DStateValidityChecker::isInBounds(double x, double y) const {
    return x >= bounds_.low[0] + agentRadius_ && x <= bounds_.high[0] - agentRadius_
        && y >= bounds_.low[1] + agentRadius_ && y <= bounds_.high[1] - agentRadius_;
}
}