//
// Created by francesco on 25.07.21.
//

#ifndef BT_ROBOTICS_TIMENDGOAL_H
#define BT_ROBOTICS_TIMENDGOAL_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace nd {

class TimeNDGoal : public ob::GoalSampleableRegion  {

public:
    TimeNDGoal(const ompl::base::SpaceInformationPtr &si, int dim);

    double distanceGoal(const ompl::base::State *st) const override;

    void sampleGoal(ompl::base::State *st) const override;

    unsigned int maxSampleCount() const override;

    /** \brief Set the goal state */
    void setState(const ob::ScopedState<> &st);

protected:
    /** \brief The goal state */
    ob::State *state_;
    ob::RealVectorStateSpace vectorStateSpace_;
};
}




#endif //BT_ROBOTICS_TIMENDGOAL_H
