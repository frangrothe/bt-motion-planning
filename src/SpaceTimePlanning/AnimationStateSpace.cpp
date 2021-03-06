//
// Created by francesco on 27.05.21.
//

#include "AnimationStateSpace.h"

namespace space_time {

AnimationStateSpace::AnimationStateSpace(const ob::StateSpacePtr& spaceComponent, double vMax, double timeWeight) : vMax_(vMax)
{
    if (timeWeight < 0 || timeWeight > 1)
        throw ompl::Exception("Error in AnimationStateSpace Construction: Time weight must be between 0 and 1");

    setName("AnimationStateSpace" + getName());
    addSubspace(spaceComponent, (1 - timeWeight)); // space component
    addSubspace(std::make_shared<ob::TimeStateSpace>(), timeWeight); // time component
    lock();
}

double AnimationStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const {
    double deltaSpace = distanceSpace(state1, state2);
    double deltaTime = distanceTime(state1, state2);

    if (deltaSpace / vMax_ > deltaTime + eps_) return std::numeric_limits<double>::infinity();

    return weights_[0] * deltaSpace + weights_[1] * deltaTime;

}

/*
 * Direction-independent distance in space
 */
double AnimationStateSpace::distanceSpace(const ompl::base::State *state1, const ompl::base::State *state2) const {
    const auto *cstate1 = static_cast<const ob::CompoundState *>(state1);
    const auto *cstate2 = static_cast<const ob::CompoundState *>(state2);

    return components_[0]->distance(cstate1->components[0], cstate2->components[0]);
}

/*
 * Direction-independent distance in time
 */
double AnimationStateSpace::distanceTime(const ompl::base::State *state1, const ompl::base::State *state2) const {
    const auto *cstate1 = static_cast<const ob::CompoundState *>(state1);
    const auto *cstate2 = static_cast<const ob::CompoundState *>(state2);

    return components_[1]->distance(cstate1->components[1], cstate2->components[1]);
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

ob::StateSpacePtr AnimationStateSpace::getSpaceComponent() {
    return components_[0];
}

ob::TimeStateSpace * AnimationStateSpace::getTimeComponent() {
    return components_[1]->as<ob::TimeStateSpace>();
}

bool AnimationStateSpace::isMetricSpace() const {
    return false;
}

double AnimationStateSpace::getMaximumExtent() const {
    return std::numeric_limits<double>::infinity();
}

void AnimationStateSpace::updateEpsilon() {
    auto extent = getTimeComponent()->isBounded() ? getTimeComponent()->getMaximumExtent() : getSpaceComponent()->getMaximumExtent() / vMax_;
    eps_ = std::numeric_limits<float>::epsilon() * std::pow(10, std::ceil(std::log10(extent)));
}

double AnimationStateSpace::getStateTime(const ompl::base::State *state) {
    return state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
}
}