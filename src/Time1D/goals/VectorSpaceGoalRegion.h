//
// Created by francesco on 04.07.21.
//

#ifndef BT_ROBOTICS_VECTORSPACEGOALREGION_H
#define BT_ROBOTICS_VECTORSPACEGOALREGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <utility>

namespace ob = ompl::base;
namespace time_1d {

class VectorSpaceGoalRegion : public ob::GoalSampleableRegion {

public:
    VectorSpaceGoalRegion(const ompl::base::SpaceInformationPtr &si, const std::vector<std::pair<double, double>> &goals);

    double distanceGoal(const ompl::base::State *st) const override;

    void sampleGoal(ompl::base::State *st) const override;

    unsigned int maxSampleCount() const override;

protected:
    class VectorSpaceValidStateSampler : public ob::ValidStateSampler {
    public:
        VectorSpaceValidStateSampler(const ompl::base::SpaceInformation *si, const std::vector<std::pair<double, double>> &goals)
                : ValidStateSampler(si), goals_(goals) {};

        bool sample(ob::State *state) override {
            int i = 0;
            if (goals_.size() > 1)
                i = rng_.uniformInt(0, goals_.size() - 1);
            double x;
            if (goals_[i].first == goals_[i].second)
                x = goals_[i].first;
            else
                x = rng_.uniformReal(goals_[i].first, goals_[i].second);
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
            return true;
        }

        bool sampleNear(ob::State *state, const ob::State *near, double distance) override {
            throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
        }

    private:
        ompl::RNG rng_;
        std::vector<std::pair<double, double>> goals_;
    };

private:
    std::vector<std::pair<double, double>> goals_;
    mutable VectorSpaceValidStateSampler sampler_; // Valid State Sampler to sample goal states

};
}


#endif //BT_ROBOTICS_VECTORSPACEGOALREGION_H
