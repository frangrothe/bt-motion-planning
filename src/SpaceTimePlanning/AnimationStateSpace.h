//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_ANIMATIONSTATESPACE_H
#define BT_ROBOTICS_ANIMATIONSTATESPACE_H

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/util/Exception.h>

namespace ob = ompl::base;
namespace space_time {

class AnimationStateSpace : public ob::CompoundStateSpace {

public:
    explicit AnimationStateSpace(const ob::StateSpacePtr& spaceComponent, double vMax = 1.0, double timeWeight = 0.5);

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

    /** \brief The time to get from state1 to state2 with respect to vMax */
    double timeToCoverDistance(const ompl::base::State *state1, const ompl::base::State *state2) const;

    /** \brief The distance of just the space component */
    double distanceSpace(const ompl::base::State *state1, const ompl::base::State *state2) const;

    /** \brief The distance of just the time component */
    double distanceTime(const ompl::base::State *state1, const ompl::base::State *state2) const;

    void setTimeBounds(double lb, double ub);
    double getVMax() const;
    void setVMax(double vMax);
    ob::StateSpacePtr getSpaceComponent();
    ob::TimeStateSpace * getTimeComponent();

    bool isMetricSpace() const override;

protected:
    double vMax_; // maximum velocity
};
}

#endif //BT_ROBOTICS_ANIMATIONSTATESPACE_H
