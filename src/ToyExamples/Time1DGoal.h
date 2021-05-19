//
// Created by francesco on 19.05.21.
//

#ifndef BT_ROBOTICS_TIME1DGOAL_H
#define BT_ROBOTICS_TIME1DGOAL_H

#include <ompl/base/Goal.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace t1d {

class Time1DGoal : public ob::Goal {

public:
    Time1DGoal(const ompl::base::SpaceInformationPtr &si, double x);

    bool isSatisfied(const ompl::base::State *st) const override;

private:
    double x_;

};
}




#endif //BT_ROBOTICS_TIME1DGOAL_H
