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

    /** \brief The distance from state1 to state2. May be infinite. */
    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

    /** \brief The time to get from state1 to state2 with respect to vMax. */
    double timeToCoverDistance(const ompl::base::State *state1, const ompl::base::State *state2) const;

    /** \brief The distance of just the space component. */
    double distanceSpace(const ompl::base::State *state1, const ompl::base::State *state2) const;

    /** \brief The distance of just the time component. */
    double distanceTime(const ompl::base::State *state1, const ompl::base::State *state2) const;

    /** \brief The time value of the given state. */
    static double getStateTime(const ompl::base::State *state) ;

    /** \brief Set lower and upper time bounds for the time component. */
    void setTimeBounds(double lb, double ub);

    double getVMax() const;
    void setVMax(double vMax);

    /** \brief The space component as a StateSpacePtr. */
    ob::StateSpacePtr getSpaceComponent();

    /** \brief The time component as a TimeStateSpace pointer. */
    ob::TimeStateSpace * getTimeComponent();

    /** \brief No metric state space, as the triangle inequality is not satisfied. */
    bool isMetricSpace() const override;

    /** \brief Maximum extent is infinite, as the distance can be infinite even with bounded time. */
    double getMaximumExtent() const override;

    /** \brief Scale epsilon appropriately after time or space bounds were set. */
    void updateEpsilon();

protected:
    double vMax_; // maximum velocity
    double eps_ = std::numeric_limits<float>::epsilon(); // epsilon for time distance
};
}

#endif //BT_ROBOTICS_ANIMATIONSTATESPACE_H
