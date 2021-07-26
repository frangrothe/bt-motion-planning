//
// Created by francesco on 19.07.21.
//

#ifndef BT_ROBOTICS_MQ2DGOAL_H
#define BT_ROBOTICS_MQ2DGOAL_H

#include <ompl/base/goals/GoalSampleableRegion.h>

namespace ob = ompl::base;
namespace query2d {

class MQ2DGoal : public ob::GoalSampleableRegion {

public:
    MQ2DGoal(const ompl::base::SpaceInformationPtr &si, double x, double y);

    double distanceGoal(const ompl::base::State *st) const override;

    void sampleGoal(ompl::base::State *st) const override;

    unsigned int maxSampleCount() const override;

private:
    double x_;
    double y_;

};
}




#endif //BT_ROBOTICS_MQ2DGOAL_H
