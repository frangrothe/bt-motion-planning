//
// Created by francesco on 18.06.21.
//

#ifndef BT_ROBOTICS_SPACETIMERRT_H
#define BT_ROBOTICS_SPACETIMERRT_H


#include <fstream>
#include <boost/math/constants/constants.hpp>
#include <chrono>

#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/util/GeometricEquations.h>
#include <ompl/base/OptimizationObjective.h>

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

    /** \brief Set whether the planner should optimize the solution with respect to time. */
    void setOptimize(bool optimize)
    {
        optimize_ = optimize;
    }

    /** \brief Get whether the planner optimizes the solution. */
    bool getOptimize() const
    {
        return optimize_;
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
        return initialBatchSize_;
    }

    void setBatchSize(int v)
    {
        if (v < 1) {
            OMPL_ERROR("%s: Batch Size needs to be at least 1.", getName().c_str());
        }
        initialBatchSize_ = v;
    }

    void setTimeBoundFactorIncrease(double f)
    {
        if (f <= 1.0) {
            OMPL_ERROR("%s: Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
            OMPL_ERROR("%s: Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
        }
        timeBoundFactorIncrease_ = f;
    }

    void setInitialTimeBoundFactor(double f)
    {
        if (f <= 1.0) {
            OMPL_ERROR("%s: Initial Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
        }
        initialTimeBoundFactor_ = f;
    }

    void setSampleUniformForUnboundedTime(bool uniform)
    {
        sampleUniformForUnboundedTime_ = uniform;
    }

    double getTimeToFirstSolution() const
    {
        return timeToFirstSolution_;
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

    /** \brief Optimization Objective to minimize time at which any goal can be achieved.
     * It is not possible to set a different optimization objective. */
    class MinimizeGoalTime : public ob::OptimizationObjective
    {
    public:
        explicit MinimizeGoalTime(const ob::SpaceInformationPtr &si) : OptimizationObjective(si) {}

    private:
        ob::Cost stateCost(const ompl::base::State *s) const override
        {
            return ob::Cost(s->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position);
        }

        ob::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
        {
            return combineCosts(stateCost(s1), stateCost(s2));
        }

        ob::Cost combineCosts(ompl::base::Cost c1, ompl::base::Cost c2) const override
        {
            return c1.value() > c2.value() ? c1 : c2;
        }

        ob::Cost identityCost() const override
        {
            return ob::Cost(-std::numeric_limits<double>::infinity());
        }

    };

    class ConditionalSampler : public ob::ValidStateSampler
    {

    public:
        ConditionalSampler(const ompl::base::SpaceInformation *si, Motion * &startMotion,
                           std::vector<Motion *> &goalMotions, std::vector<Motion *> &newlyAddedGoalMotions,
                           bool &sampleOldBatch)
                : ValidStateSampler(si),
                  startMotion_(startMotion), goalMotions_(goalMotions), newBatchGoalMotions_(newlyAddedGoalMotions),
                  sampleOldBatch_(sampleOldBatch)
        {
            name_ = "ConditionalSampler";
        }

        bool sample(ob::State *state) override {
            bool validSample = false;

            while (!validSample) {
                internalSampler_->sampleUniform(state);
                double leftBound, rightBound;
                // get minimum time, when the state can be reached from the start
                double startBound = startMotion_->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position +
                                   si_->getStateSpace()->as<space_time::AnimationStateSpace>()->timeToCoverDistance(state, startMotion_->state);
                // sample old batch
                if (sampleOldBatch_) {
                    leftBound = startBound;
                    // get maximum time, at which any goal can be reached from the state
                    rightBound = std::numeric_limits<double>::min();
                    for (auto goal : goalMotions_) {
                        double t = goal->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                                   si_->getStateSpace()->as<space_time::AnimationStateSpace>()->timeToCoverDistance(goal->state, state);
                        if (t > rightBound) {
                            rightBound = t;
                        }
                    }
                }
                // sample new batch
                else {
                    // get maximum time, at which any goal from the new batch can be reached from the state
                    rightBound = std::numeric_limits<double>::min();
                    for (auto goal : newBatchGoalMotions_) {
                        double t = goal->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                                   si_->getStateSpace()->as<space_time::AnimationStateSpace>()->timeToCoverDistance(goal->state, state);
                        if (t > rightBound) {
                            rightBound = t;
                        }
                    }
                    // get maximum time, at which any goal from the old batch can be reached from the state
                    // only allow the left bound to be smaller than the right bound
                    leftBound = std::numeric_limits<double>::min();
                    for (auto goal : goalMotions_) {
                        double t = goal->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                                   si_->getStateSpace()->as<space_time::AnimationStateSpace>()->timeToCoverDistance(goal->state, state);
                        if (t > leftBound && t < rightBound) {
                            leftBound = t;
                        }
                    }
                    leftBound = std::max(leftBound, startBound);
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

        std::vector<Motion *> &goalMotions_;
        std::vector<Motion *> &newBatchGoalMotions_;

        /** References to whether the old or new batch region is sampled */
        bool &sampleOldBatch_;

        /** \brief The random number generator */
        ompl::RNG rng_;
    };

    /** \brief Whether the solution is optimized for time or the first solution terminates the algorithm. */
    bool optimize_ = true;

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

    /** \brief Gets the neighbours of a given motion, using either k-nearest or radius_ as appropriate. */
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
    ob::PathPtr bestSolution_{nullptr};

    /** \brief The current best time i.e. cost of all found solutions */
    double bestTime_ = std::numeric_limits<double>::infinity();

    /** \brief The number of while loop iterations */
    unsigned int numIterations_ = 0;

    /** \brief The number of found solutions */
    int numSolutions_ = 0;

    /** \brief Minimum Time at which any goal can be reached, if moving on a straight line. */
    double minimumTime_ = std::numeric_limits<double>::infinity();

    /** \brief Upper bound for the time up to which solutions are searched for. */
    double upperTimeBound_;

    /** \brief The factor to which found solution times need to be reduced compared to minimum time, (0, 1]. */
    double optimumApproxFactor_ = 1.0;

    /** \brief The start Motion, used for conditional sampling and start tree pruning. */
    Motion * startMotion_{nullptr};

    /** \brief The goal Motions, used for conditional sampling and pruning. */
    std::vector<Motion *> goalMotions_{};

    /** \brief The goal Motions, that were added in the current expansion step, used for uniform sampling over a growing region. */
    std::vector<Motion *> newBatchGoalMotions_{};

    /**
     * Goal Sampling is not handled by PlannerInputStates, but directly by the SpaceTimeRRT,
     * because the time component of every goal sample is sampled dependent on the sampled space component.
     *
     */

    ob::State *tempState_{nullptr}; // temporary sampled goal states are stored here.

    /** \brief N tries to sample a goal. */
    ob::State * nextGoal(int n, double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor);

    /** \brief Samples a goal until successful or the termination condition is fulfilled. */
    ob::State * nextGoal(const ob::PlannerTerminationCondition &ptc, double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor);

    /** \brief Samples a goal until successful or the termination condition is fulfilled. */
    ob::State * nextGoal(const ob::PlannerTerminationCondition &ptc, int n, double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor);

    /** \brief Samples the time component of a goal state dependant on its space component. Returns false, if goal can't be reached in time. */
    bool sampleGoalTime(ob::State * goal, double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor);

    /** \brief Removes the given motion from the parent's child list. */
    static void removeFromParent(Motion *m);

    /** \brief Adds given all descendants of the given motion to given tree and checks whether one of the added motions is the goal motion. */
    static void addDescendants(Motion *m, const TreeData &tree);

    void constructSolution(Motion *startMotion, Motion *goalMotion, const ob::ReportIntermediateSolutionFn &intermediateSolutionCallback);

    enum RewireState {
        // use r-disc search for rewiring
        RADIUS,
        // use k-nearest for rewiring
        KNEAREST,
        // don't use any rewiring
        OFF
    };

    RewireState rewireState_ = KNEAREST;

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

    /** \brief The time bound the planner is initialized with. Used to reset for repeated planning */
    double initialTimeBound_;

    /** \brief Number of samples of the first batch */
    unsigned int initialBatchSize_ = 512;

    /** \brief Initial factor, the minimum time of each goal is multiplied with to calculate the upper time bound. */
    double initialTimeBoundFactor_ = 2.0;

    /** \brief The factor, the time bound is increased with after the batch is full. */
    double timeBoundFactorIncrease_ = 2.0;

    bool sampleOldBatch_ = true;

    /** \brief Whether the samples are uniformly distributed over the whole space or are centered at lower times. */
    bool sampleUniformForUnboundedTime_ = true;

    /** \brief The ratio, a goal state is sampled compared to the size of the goal tree. */
    int goalStateSampleRatio_ = 4;

    /** \brief The random number generator */
    ompl::RNG rng_;

    std::chrono::time_point<std::chrono::steady_clock> startTime_;
    double timeToFirstSolution_ = std::numeric_limits<double>::infinity();

    ///////////////////////////////////////
    // Planner progress property functions
    std::string numIterationsProperty() const
    {
        return std::to_string(numIterations_);
    }
    std::string bestCostProperty() const
    {
        return std::to_string(bestTime_);
    }

    void writeSamplesToCSV(const ob::PlannerData &data, int n);
    void writePathToCSV(const ob::PathPtr &path);
    std::vector<int> samplesToDraw_{
        5, 10, 20, 50, 100, 250
    };

};

}




#endif //BT_ROBOTICS_SPACETIMERRT_H
