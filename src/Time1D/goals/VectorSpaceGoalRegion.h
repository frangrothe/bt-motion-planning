//
// Created by francesco on 04.07.21.
//

#ifndef BT_ROBOTICS_VECTORSPACEGOALREGION_H
#define BT_ROBOTICS_VECTORSPACEGOALREGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

namespace ob = ompl::base;
namespace time_1d {

class VectorSpaceGoalRegion : public ob::GoalSampleableRegion {

public:
    VectorSpaceGoalRegion(const ompl::base::SpaceInformationPtr &si, double minX, double maxX);

    double distanceGoal(const ompl::base::State *st) const override;

    void sampleGoal(ompl::base::State *st) const override;

    unsigned int maxSampleCount() const override;

protected:
    class VectorSpaceValidStateSampler : public ob::ValidStateSampler {
    public:
        VectorSpaceValidStateSampler(const ompl::base::SpaceInformation *si, double minX, double maxX)
                : ValidStateSampler(si), minX_(minX),
                  maxX_(maxX) {};

        bool sample(ob::State *state) override {
            double x = rng_.uniformReal(minX_,maxX_);
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
            return true;
        }

        bool sampleNear(ob::State *state, const ob::State *near, double distance) override {
            throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
        }

    private:
        ompl::RNG rng_;
        double minX_;
        double maxX_;
    };

private:
    double minX_;
    double maxX_;
    mutable VectorSpaceValidStateSampler sampler_; // Valid State Sampler to sample goal states

};
}


#endif //BT_ROBOTICS_VECTORSPACEGOALREGION_H
