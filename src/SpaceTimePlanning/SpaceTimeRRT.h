//
// Created by francesco on 18.06.21.
//

#ifndef BT_ROBOTICS_SPACETIMERRT_H
#define BT_ROBOTICS_SPACETIMERRT_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include "AnimationStateSpace.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace space_time {

class SpaceTimeRRT : public ob::Planner {

public:
    /** \brief Constructor */
    explicit SpaceTimeRRT(const ompl::base::SpaceInformationPtr &si);

    ~SpaceTimeRRT() override;

    void clear() override;

    void getPlannerData(ob::PlannerData &data) const override;

    ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

    /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
    void setRange(double distance)
    {
        maxDistance_ = distance;
    }

    /** \brief Get the range the planner is using */
    double getRange() const
    {
        return maxDistance_;
    }

    void setup() override;

protected:
    /** \brief Representation of a motion */
    class Motion
    {
    public:
        Motion() = default;

        explicit Motion(const ob::SpaceInformationPtr &si) : state(si->allocState())
        {
        }

        ~Motion() = default;

        const ob::State *root{nullptr};
        ob::State *state{nullptr};
        Motion *parent{nullptr};
    };

    class ConditionalSampler : public ob::ValidStateSampler
    {
    public:
        explicit ConditionalSampler(const ompl::base::SpaceInformation *si) : ValidStateSampler(si)
        {
            name_ = "ConditionalSampler";
        }

        bool sample(ob::State *state) override {
            bool validSample = false;
            while (!validSample) {
                internalSampler_->sampleUniform(state);

                // get minimum time, when the state can be reached from any start
                double leftBound = std::numeric_limits<double>::max();
                for (auto start : startStates_) {
                    double t = start->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position +
                               si_->getStateSpace()->template as<space_time::AnimationStateSpace>()->timeToCoverDistance(state, start);
                    if (t < leftBound) {
                        leftBound = t;
                    }
                }

                // get maximum time, when any goal can be reached from the state
                double rightBound = std::numeric_limits<double>::min();
                for (auto goal : goalStates_) {
                    double t = goal->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                               si_->getStateSpace()->template as<space_time::AnimationStateSpace>()->timeToCoverDistance(state, goal);
                    if (t > rightBound) {
                        rightBound = t;
                    }
                }

                if (leftBound <= rightBound) {
                    double time = rng_.uniformReal(leftBound, rightBound);
                    state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = time;
                    validSample = true;
                }
            }
            return true;
        }

        bool sampleNear(ob::State *state, const ob::State *near, const double distance) override {
            throw ompl::Exception("ConditionalSampler::sampleNear", "not implemented");
        }

        void addStartState(ob::State* s) {startStates_.push_back(s);}
        void addGoalState(ob::State* s) {goalStates_.push_back(s);}

        void clear()
        {
            if (!startStates_.empty())
            {
                for (auto s : startStates_) {
                    si_->freeState(s);
                }
            }

            if (!goalStates_.empty())
            {
                for (auto s : goalStates_) {
                    si_->freeState(s);
                }
            }
        }

    private:
        ob::StateSamplerPtr internalSampler_ = si_->allocStateSampler();
        std::vector<ob::State*> startStates_;
        std::vector<ob::State*> goalStates_;

        /** \brief The random number generator */
        ompl::RNG rng_;
    };

    /** \brief A nearest-neighbor datastructure representing a tree of motions */
    using TreeData = std::shared_ptr<ompl::NearestNeighbors<Motion *>>;

    /** \brief Information attached to growing a tree of motions (used internally) */
    struct TreeGrowingInfo
    {
        ob::State *xstate;
        Motion *xmotion;
        bool start;
    };

    /** \brief The state of the tree after an attempt to extend it */
    enum GrowState
    {
        /// no progress has been made
        TRAPPED,
        /// progress has been made towards the randomly sampled state
        ADVANCED,
        /// the randomly sampled state was reached
        REACHED
    };

    /** \brief Grow a tree towards a random state */
    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

    /** \brief Free the memory allocated by this planner */
    void freeMemory();

    /** \brief Compute distance between motions (actually distance between contained states) */
    double distanceFunction(const Motion *a, const Motion *b) const
    {
        return si_->distance(a->state, b->state);
    }

    /** \brief State sampler */
    ConditionalSampler sampler_;

    /** \brief The start tree */
    TreeData tStart_;

    /** \brief The goal tree */
    TreeData tGoal_;

    /** \brief The maximum length of a motion to be added to a tree */
    double maxDistance_{0.};

    /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
    std::pair<ob::State *, ob::State *> connectionPoint_;

    /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
    double distanceBetweenTrees_;

};
}




#endif //BT_ROBOTICS_SPACETIMERRT_H
