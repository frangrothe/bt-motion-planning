//
// Created by francesco on 18.08.21.
//

#include "MinimizeArrivalTime.h"

space_time::MinimizeArrivalTime::MinimizeArrivalTime(const ompl::base::SpaceInformationPtr &si) : OptimizationObjective(
        si) {}

ompl::base::Cost space_time::MinimizeArrivalTime::stateCost(const ompl::base::State *s) const {
    return ob::Cost(s->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position);
}

ompl::base::Cost
space_time::MinimizeArrivalTime::motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const {
    return combineCosts(stateCost(s1), stateCost(s2));
}

ompl::base::Cost space_time::MinimizeArrivalTime::combineCosts(ompl::base::Cost c1, ompl::base::Cost c2) const {
    return c1.value() > c2.value() ? c1 : c2;
}

ompl::base::Cost space_time::MinimizeArrivalTime::identityCost() const {
    return ob::Cost(-std::numeric_limits<double>::infinity());
}

