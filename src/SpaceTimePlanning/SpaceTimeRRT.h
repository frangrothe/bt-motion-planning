//
// Created by francesco on 18.06.21.
//

#ifndef BT_ROBOTICS_SPACETIMERRT_H
#define BT_ROBOTICS_SPACETIMERRT_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
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
        /** \brief The set of motions descending from the current motion */
        std::vector<Motion *> children;
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
                // get minimum time, when the state can be reached from the start
                double leftBound = startState_->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position +
                                   si_->getStateSpace()->template as<space_time::AnimationStateSpace>()->timeToCoverDistance(state, startState_);

                // get maximum time, at which any goal can be reached from the state
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

        void addStartState(ob::State* s) {startState_ = s;}
        void addGoalState(ob::State* s) {goalStates_.push_back(s);}

        void clear()
        {
            if (startState_ != nullptr)
            {
                si_->freeState(startState_);
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
        ob::State* startState_;
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

    int pruneStartTree();

    std::tuple<int, bool> pruneGoalTree(Motion * goalMotion);

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

    /** \brief The current best solution path with respect to shortest time. */
    ob::PathPtr bestSolution_;

    /** \brief Minimum Time at which any goal can be reached, if moving on a straight line. */
    double minimumTime_ = std::numeric_limits<double>::infinity();

    /** \brief Upper bound for the time up to which solutions are searched for. */
    double upperTimeBound_;

    /** \brief The factor to which found solution times need to be reduced compared to minimum time, (0, 1). */
    double solutionImproveFactor_ = 0.9;

    /** \brief The difference, at which to times are considered equal. */
    double epsilon_ = 0.0001;

    /** \brief The start Motion, used for start tree pruning. */
    Motion * startMotion_;

    static bool goalSetCmp(Motion * a, Motion * b) {
        return a->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position <
                b->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    }
    /** \brief The goal Motions, ordered by time ascending. */
    std::set<Motion *, decltype(&goalSetCmp)> goalSet_;

    /**
     * Gaol Sampling is not handled by PlannerInputStates, but directly by the SpaceTimeRRT,
     * because the time component of every goal sample is sampled dependant on the sampled space component.
     *
     */

    ob::State *tempState_; // temporary sampled goal states are stored here.

    /** \brief A single try to sample a goal. */
    ob::State * nextGoal();

    /** \brief Samples a goal until successful or the termination condition is fulfilled. */
    ob::State * nextGoal(const ob::PlannerTerminationCondition &ptc);

    /** \brief Samples the time component of a goal state dependant on its space component. Returns false, if goal can't be reached in time. */
    bool sampleGoalTime(ob::State * goal);

    /** \brief Removes the given motion from the parent's child list. */
    static void removeFromParent(Motion *m);

    /** \brief Adds given all descendants of the given motion to given tree and checks whether one of the added motions is the goal motion. */
    static void addDescendants(Motion *m, const TreeData &tree, Motion *goalMotion, bool *addedGoalMotion);

    void constructSolution(Motion *startMotion, Motion *goalMotion);

    void writeSamplesToCSV(const std::string& num);

    /** \brief The random number generator */
    ompl::RNG rng_;

};
}




#endif //BT_ROBOTICS_SPACETIMERRT_H
