//
// Created by francesco on 18.08.21.
//

#ifndef BT_ROBOTICS_MINIMIZEARRIVALTIME_H
#define BT_ROBOTICS_MINIMIZEARRIVALTIME_H

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/TimeStateSpace.h>

namespace ob = ompl::base;
namespace space_time {

class MinimizeArrivalTime : public ob::OptimizationObjective {

public:
    explicit MinimizeArrivalTime(const ompl::base::SpaceInformationPtr &si);

private:
    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    ompl::base::Cost combineCosts(ompl::base::Cost c1, ompl::base::Cost c2) const override;

    ompl::base::Cost identityCost() const override;

};
}


#endif //BT_ROBOTICS_MINIMIZEARRIVALTIME_H
