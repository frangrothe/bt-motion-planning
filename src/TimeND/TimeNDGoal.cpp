//
// Created by francesco on 25.07.21.
//

#include "TimeNDGoal.h"

namespace nd {


TimeNDGoal::TimeNDGoal(const ompl::base::SpaceInformationPtr &si, int dim) : GoalSampleableRegion(si),
                                                                             state_(nullptr),
                                                                             vectorStateSpace_(dim) {}

void TimeNDGoal::setState(const ob::ScopedState<> &st) {
    if (state_ != nullptr)
        si_->freeState(state_);
    state_ = si_->cloneState(st.get());
}

double TimeNDGoal::distanceGoal(const ompl::base::State *st) const {
    return vectorStateSpace_.distance(st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0),
            state_->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0));

}

void TimeNDGoal::sampleGoal(ompl::base::State *st) const {
    si_->copyState(st, state_);
}

unsigned int TimeNDGoal::maxSampleCount() const {
    return 1;
}
}