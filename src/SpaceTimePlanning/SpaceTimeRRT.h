//
// Created by francesco on 18.06.21.
//

#ifndef BT_ROBOTICS_SPACETIMERRT_H
#define BT_ROBOTICS_SPACETIMERRT_H


#include <fstream>
#include <boost/math/constants/constants.hpp>

#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/util/GeometricEquations.h>

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

    double getOptimumApproxFactor() const
    {
        return optimumApproxFactor_;
    }

    void setOptimumApproxFactor(double optimumApproxFactor)
    {
        if (optimumApproxFactor <= 0 || optimumApproxFactor > 1) {
            OMPL_ERROR("%s: The optimum approximation factor needs to be between 0 and 1.", getName().c_str());
        }
        optimumApproxFactor_ = optimumApproxFactor;
    }

    std::string getRewiringState() const
    {
        std::vector<std::string> s{"Radius", "KNearest", "Off"};
        return s[rewireState_];
    }

    void setRewiringToOff()
    {
        rewireState_ = OFF;
    }

    void setRewiringToRadius()
    {
        rewireState_ = RADIUS;
    }

    void setRewiringToKNearest()
    {
        rewireState_ = KNEAREST;
    }

    double getRewireFactor() const
    {
        return rewireFactor_;
    }

    void setRewireFactor(double v)
    {
        if (v <= 1) {
            OMPL_ERROR("%s: Rewire Factor needs to be greater than 1.", getName().c_str());
        }
        rewireFactor_ = v;
    }

    unsigned int getBatchSize() const
    {
        return batchSize_;
    }

    void setBatchSize(int v)
    {
        if (v < 1) {
            OMPL_ERROR("%s: Batch Size needs to be at least 1.", getName().c_str());
        }
        batchSize_ = v;
    }

    void setup() override;

protected:
    /** \brief Representation of a motion */
    class Motion
    {
    public:
        Motion() = default;

        explicit Motion(const ob::SpaceInformationPtr &si) : state(si->allocState()) {}

        ~Motion() = default;

        const ob::State *root{nullptr};
        ob::State *state{nullptr};
        Motion *parent{nullptr};
        /** \brief The set of motions descending from the current motion */
        std::vector<Motion *> children{};
        // only used by goal tree
        Motion *connectionPoint{nullptr}; // the start tree motion, if there is a direct connection
        int numConnections{0}; // number of connections to the start tree of self and all descendants
    };

    class ConditionalSampler : public ob::ValidStateSampler
    {
        static bool goalSetCmp(Motion * a, Motion * b) {
            return a->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position <
                   b->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        }
    public:
        ConditionalSampler(const ompl::base::SpaceInformation *si, Motion * &startMotion,
                                    std::set<Motion *, decltype(&goalSetCmp)> &goalSet) : ValidStateSampler(si),
                                    startMotion_(startMotion), goalSet_(goalSet)
        {
            name_ = "ConditionalSampler";
        }

        bool sample(ob::State *state) override {
            bool validSample = false;
            while (!validSample) {
                internalSampler_->sampleUniform(state);
                // get minimum time, when the state can be reached from the start
                double leftBound = startMotion_->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position +
                                   si_->getStateSpace()->as<space_time::AnimationStateSpace>()->timeToCoverDistance(state, startMotion_->state);

                // get maximum time, at which any goal can be reached from the state
                double rightBound = std::numeric_limits<double>::min();
                for (auto goal : goalSet_) {
                    double t = goal->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                               si_->getStateSpace()->as<space_time::AnimationStateSpace>()->timeToCoverDistance(goal->state, state);
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

    private:
        ob::StateSamplerPtr internalSampler_ = si_->allocStateSampler();

        /** References to the start state and goal states */
        Motion * &startMotion_;

        std::set<Motion *, decltype(&goalSetCmp)> &goalSet_;

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

    /** \brief Grow a tree towards a random state for a single nearest state */
    GrowState growTreeSingle(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion, Motion *nmotion);

    /** \brief Attempt to grow a tree towards a random state for the neighborhood of the random state. The
     * neighborhood is determined by the used rewire state. For the start tree closest state with respect to distance are tried first.
     * For the goal tree states with the minimum time root node are tried first. If connect is true, multiple vertices can be added to the tree
     * until the random state is reached or an obstacle is met. If connect is false, the tree is only extended by a single new state. */
    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion, std::vector<Motion *> &nbh, bool connect);

    /** \brief Gets the neighbours of a given motion, using either k-nearest or radius as appropriate. */
    void getNeighbors(TreeData &tree, Motion *motion, std::vector<Motion *> &nbh) const;

    /** \brief Free the memory allocated by this planner */
    void freeMemory();

    /** \brief Compute distance between motions (actually distance between contained states) */
    double distanceFunction(const Motion *a, const Motion *b) const
    {
        return si_->distance(a->state, b->state);
    }

    /** \brief Prune the start tree after a solution was found. */
    void pruneStartTree();

    /** \brief Prune the goal tree after a solution was found.
     * Return the goal motion, that is connected to the start tree, if a new solution was found.
     * If no new solution was found, return nullpointer. */
    Motion* pruneGoalTree();

    /** \brief State sampler */
    ConditionalSampler sampler_;

    /** \brief The start tree */
    TreeData tStart_;

    /** \brief The goal tree */
    TreeData tGoal_;

    /** \brief The maximum length of a motion to be added to a tree */
    double maxDistance_{0.};

    /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
    double distanceBetweenTrees_;

    /** \brief The current best solution path with respect to shortest time. */
    ob::PathPtr bestSolution_;

    /** \brief The number of found solutions */
    int numSolutions = 0;

    /** \brief Minimum Time at which any goal can be reached, if moving on a straight line. */
    double minimumTime_ = std::numeric_limits<double>::infinity();

    /** \brief Upper bound for the time up to which solutions are searched for. */
    double upperTimeBound_;

    /** \brief The factor to which found solution times need to be reduced compared to minimum time, (0, 1]. */
    double optimumApproxFactor_ = 1.0;

    /** \brief The difference, at which two doubles are considered equal. */
    double epsilon_ = 1.0e-9;

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
    static void addDescendants(Motion *m, const TreeData &tree);

    void constructSolution(Motion *startMotion, Motion *goalMotion);

    enum RewireState {
        // use r-disc search for rewiring
        RADIUS,
        // use k-nearest for rewiring
        KNEAREST,
        // don't use any rewiring
        OFF
    };

    RewireState rewireState_ = OFF;

    /** \brief The rewiring factor, s, so that r_rrt = s \times r_rrt* > r_rrt* (or k_rrt = s \times k_rrt* >
             * k_rrt*) */
    double rewireFactor_{1.1};

    /** \brief A constant for k-nearest rewiring calculations */
    double k_rrt_{0u};

    /** \brief A constant for r-disc rewiring calculations */
    double r_rrt_{0.};

    /** \brief Calculate the k_RRG* and r_RRG* terms */
    void calculateRewiringLowerBounds();

    bool rewireGoalTree(Motion* addedMotion);

    /** \brief Whether the time is bounded or not. The first solution automatically bounds the time. */
    bool isTimeBounded_;

    /** \brief Number of samples before the upper time bound gets increased */
    unsigned int batchSize_ = 1000;

    /** \brief While time is unbounded, goals are sampled from their respective minimum time to minimum time \times time bound factor. */
    double timeBoundFactor_ = 2;

    void writeSamplesToCSV(const std::string& type);
    void writeSolutionToCSV();

    /** \brief The random number generator */
    ompl::RNG rng_;

};
}




#endif //BT_ROBOTICS_SPACETIMERRT_H
