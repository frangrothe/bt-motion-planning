//
// Created by francesco on 27.05.21.
//

#include "AnimationStateSpace.h"

namespace space_time {

AnimationStateSpace::AnimationStateSpace(unsigned int dim, double vMax, double timeWeight) : vMax_(vMax) {
    if (timeWeight < 0 || timeWeight > 1)
        throw ompl::Exception("Error in AnimationStateSpace Construction: Time weight must be between 0 and 1");

    setName("AnimationStateSpace" + getName());
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(dim), (1 - timeWeight));
    addSubspace(std::make_shared<ob::TimeStateSpace>(), timeWeight);
    lock();
}

double AnimationStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const {
    double deltaSpace = distanceSpace(state1, state2);
    double deltaTime = distanceTime(state1, state2);

    if (deltaSpace / vMax_ > deltaTime) return std::numeric_limits<double>::infinity();

    return weights_[0] * deltaSpace + weights_[1] * deltaTime;

}

double AnimationStateSpace::distanceSpace(const ompl::base::State *state1, const ompl::base::State *state2) const {
    const auto *cstate1 = dynamic_cast<const ob::CompoundState *>(state1);
    const auto *cstate2 = dynamic_cast<const ob::CompoundState *>(state2);

    return components_[0]->distance(cstate1->components[0], cstate2->components[0]);

}

double AnimationStateSpace::distanceTime(const ompl::base::State *state1, const ompl::base::State *state2) const {
    const auto *cstate1 = dynamic_cast<const ob::CompoundState *>(state1);
    const auto *cstate2 = dynamic_cast<const ob::CompoundState *>(state2);

    return components_[1]->distance(cstate1->components[1], cstate2->components[1]);
}

void AnimationStateSpace::setVectorBounds(const ob::RealVectorBounds &bounds) {
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

void AnimationStateSpace::setTimeBounds(double lb, double ub) {
    as<ob::TimeStateSpace>(1)->setBounds(lb, ub);
}

double AnimationStateSpace::getVMax() const {
    return vMax_;
}

void AnimationStateSpace::setVMax(double vMax) {
    vMax_ = vMax;
}

double AnimationStateSpace::timeToCoverDistance(const ompl::base::State *state1, const ompl::base::State *state2) const {
    double deltaSpace = distanceSpace(state1, state2);
    return deltaSpace / vMax_;
}
}