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
namespace SpaceTime {

class AnimationStateSpace : public ob::CompoundStateSpace {

public:
    explicit AnimationStateSpace(unsigned int dim, double vMax = 1.0, double timeWeight = 0.5);

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

    void setVectorBounds(const ob::RealVectorBounds &bounds);
    void setTimeBounds(double lb, double ub);
    double getVMax() const;
    void setVMax(double vMax);

protected:
    double vMax_; // maximum velocity

    double distanceSpace(const ompl::base::State *state1, const ompl::base::State *state2) const;
    double distanceTime(const ompl::base::State *state1, const ompl::base::State *state2) const;

};
}

#endif //BT_ROBOTICS_ANIMATIONSTATESPACE_H
