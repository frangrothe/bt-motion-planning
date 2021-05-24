//
// Created by francesco on 24.05.21.
//

#ifndef BT_ROBOTICS_TIMERRT_H
#define BT_ROBOTICS_TIMERRT_H

#include <ompl/base/Planner.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace t1d {

class TimeRRT : public ob::Planner {

public:
    /** \brief Constructor */
    explicit TimeRRT(const ob::SpaceInformationPtr &si, bool addIntermediateStates = false);
    TimeRRT(const ompl::base::SpaceInformationPtr &si, double maxSpeed);

    ~TimeRRT() override;

    void getPlannerData(ob::PlannerData &data) const override;

    ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

    void clear() override;

    /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
     */
    bool getIntermediateStates() const
    {
        return addIntermediateStates_;
    }

    /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
     * itself */
    void setIntermediateStates(bool addIntermediateStates)
    {
        addIntermediateStates_ = addIntermediateStates;
    }

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

    /** \brief Set a different nearest neighbors datastructure */
    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        clear();
        tStart_ = std::make_shared<NN<Motion *>>();
        tGoal_ = std::make_shared<NN<Motion *>>();
        setup();
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

    /** \brief Free the memory allocated by this planner */
    void freeMemory();

    /** \brief Compute distance between motions (actually distance between contained states) */
    double distanceFunction(const Motion *a, const Motion *b) const;

    /** \brief Grow a tree towards a random state */
    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

    /** \brief State sampler */
    ob::StateSamplerPtr sampler_;

    /** \brief The start tree */
    TreeData tStart_;

    /** \brief The goal tree */
    TreeData tGoal_;

    /** \brief The maximum length of a motion to be added to a tree */
    double maxDistance_{0.};

    /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
    bool addIntermediateStates_;

    /** \brief The random number generator */
    ompl::RNG rng_;

    /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
    std::pair<ob::State *, ob::State *> connectionPoint_;

    /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
    double distanceBetweenTrees_;

    // max speed of the robot. Used for distance calculation
    double maxSpeed_;

    // The weight of the time difference for the distance calculation, value in [0, 1].
    double distanceTimeWeight_ = 0.2;
};
}





#endif //BT_ROBOTICS_TIMERRT_H
