//
// Created by francesco on 18.06.21.
//

#include "SpaceTimeRRT.h"

namespace space_time {


SpaceTimeRRT::SpaceTimeRRT(const ompl::base::SpaceInformationPtr &si)
        : Planner(si, "SpaceTimeRRT"),
          sampler_(&(*si), startMotion_, goalMotions_, newBatchGoalMotions_, sampleOldBatch_)
{

    Planner::declareParam<double>("range", this, &SpaceTimeRRT::setRange, &SpaceTimeRRT::getRange, "0.:1.:10000.");
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

SpaceTimeRRT::~SpaceTimeRRT() {
    freeMemory();
}

void SpaceTimeRRT::setup() {

    Planner::setup();
    ompl::tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(new ompl::NearestNeighborsLinear<Motion *>());
    if (!tGoal_)
        tGoal_.reset(new ompl::NearestNeighborsLinear<Motion *>());
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    if (si_->getStateSpace()->as<space_time::AnimationStateSpace>()->getTimeComponent()->isBounded()) {
        upperTimeBound_ = si_->getStateSpace()->as<space_time::AnimationStateSpace>()->getTimeComponent()->getMaxTimeBound();
        isTimeBounded_ = true;
    } else {
        upperTimeBound_ = std::numeric_limits<double>::infinity();
        isTimeBounded_ = false;
    }

    // Calculate some constants:
    calculateRewiringLowerBounds();
}

void SpaceTimeRRT::freeMemory() {
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tempState_) si_->freeState(tempState_);
}

ob::PlannerStatus SpaceTimeRRT::solve(const ob::PlannerTerminationCondition &ptc) {

    checkValidity();
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const ob::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
        startMotion_ = motion;
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    std::vector<Motion *> nbh;

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    ob::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;
    unsigned int batchSize = initialBatchSize_; // samples to fill the current batch
    int numBatchSamples = static_cast<int>(tStart_->size() + tGoal_->size()); // number of samples in the current batch (old + new batch region)
    int newBatchGoalSamples = 0; // number of goal samples in the new batch region
    bool firstBatch = true;
    double oldBatchSampleProb = 1.0; // probability to sample the old batch region

    OMPL_INFORM("%s: Starting planning with time bound factor %.2f", getName().c_str(),
                newBatchTimeBoundFactor_);

    while (!ptc)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        // batch is full
        if (!isTimeBounded_ && numBatchSamples >= batchSize) {
            if (firstBatch) {
                firstBatch = false;
                oldBatchSampleProb = 0.5 * (1 / timeBoundFactorIncrease_);
            }
            oldBatchTimeBoundFactor_ = newBatchTimeBoundFactor_;
            newBatchTimeBoundFactor_ *= timeBoundFactorIncrease_;
            startTree = true;
            batchSize = std::ceil(2.0 * (timeBoundFactorIncrease_ - 1.0) * static_cast<double>(tStart_->size() + tGoal_->size()));
            numBatchSamples = 0;
            if (!newBatchGoalMotions_.empty()) {
                goalMotions_.insert(goalMotions_.end(), newBatchGoalMotions_.begin(), newBatchGoalMotions_.end());
                newBatchGoalMotions_.clear();
            }
            OMPL_INFORM("%s: Increased time bound factor to %.2f", getName().c_str(),
                        newBatchTimeBoundFactor_);
        }

        // determine whether the old or new batch is sampled
        sampleOldBatch_ = (firstBatch || isTimeBounded_ || rng_.uniform01() <= oldBatchSampleProb);

        ob::State *goalState{nullptr};
        if (sampleOldBatch_) {
            // sample until successful or time is up
            if (goalMotions_.empty() && isTimeBounded_)
                goalState = nextGoal(ptc);
            // sample for n tries, with n = batch size
            else if (goalMotions_.empty() && !isTimeBounded_) {
                goalState = nextGoal(static_cast<int>(batchSize));
                // the goal region is most likely blocked for this time period -> increase upper time bound
                if (goalState == nullptr) {
                    newBatchTimeBoundFactor_ *= timeBoundFactorIncrease_;
                    oldBatchTimeBoundFactor_ = newBatchTimeBoundFactor_;
                    startTree = true;
                    batchSize = std::ceil(2.0 * (timeBoundFactorIncrease_ - 1.0) * static_cast<double>(tStart_->size() + tGoal_->size()));
                    numBatchSamples = 0;
                    OMPL_INFORM("%s: Increased time bound factor to %.2f", getName().c_str(),
                                newBatchTimeBoundFactor_);
                    continue;
                }
            }
            // sample for a single try
            else if (goalMotions_.size() < (tGoal_->size() - newBatchGoalSamples) / goalStateSampleRatio_)
                goalState = nextGoal(1);
        }
        else {
            if (newBatchGoalMotions_.empty()) {
                goalState = nextGoal(static_cast<int>(batchSize));
                // the goal region is most likely blocked for this time period -> increase upper time bound
                if (goalState == nullptr) {
                    oldBatchTimeBoundFactor_ = newBatchTimeBoundFactor_;
                    newBatchTimeBoundFactor_ *= timeBoundFactorIncrease_;
                    startTree = true;
                    batchSize = std::ceil(2.0 * (timeBoundFactorIncrease_ - 1.0) * static_cast<double>(tStart_->size() + tGoal_->size()));
                    numBatchSamples = 0;
                    OMPL_INFORM("%s: Increased time bound factor to %.2f", getName().c_str(),
                                newBatchTimeBoundFactor_);
                    continue;
                }
            }
            else if (newBatchGoalMotions_.size() < newBatchGoalSamples / goalStateSampleRatio_)
                goalState = nextGoal(1);
        }

        if (goalState != nullptr) {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, goalState);
            motion->root = motion->state;
            tGoal_->add(motion);
            if (sampleOldBatch_)
                goalMotions_.push_back(motion);
            else {
                newBatchGoalMotions_.push_back(motion);
                newBatchGoalSamples++;
            }

            minimumTime_ = std::min(minimumTime_, si_->getStateSpace()
                    ->as<space_time::AnimationStateSpace>()->timeToCoverDistance(startMotion_->state, goalState));
            numBatchSamples++;
        }

        /* sample random state */
        sampler_.sample(rstate);

        // EXTEND
        GrowState gs = growTree(tree, tgi, rmotion, nbh, false);
        if (gs != TRAPPED)
        {
            numBatchSamples++;
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;
            Motion *startMotion;
            Motion *goalMotion;

            /* rewire the goal tree */
            bool newSolution = false;
            if (!tgi.start && rewireState_ != OFF) {
                newSolution = rewireGoalTree(addedMotion);
                if (newSolution) {
                    // find connection point
                    std::queue<Motion*> queue;
                    queue.push(addedMotion);
                    while (!queue.empty()) {
                        if (queue.front()->connectionPoint != nullptr) {
                            goalMotion = queue.front();
                            startMotion = queue.front()->connectionPoint;
                            break;
                        }
                        else {
                            for (Motion* c : queue.front()->children)
                                queue.push(c);
                        }
                        queue.pop();
                    }
                }
            }

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            tgi.start = startTree;

            /* attempt to connect trees, if rewiring didn't find a new solution */
            // CONNECT
            if (!newSolution) {
                int totalSamples = static_cast<int>(tStart_->size() + tGoal_->size());
                GrowState gsc = growTree(otherTree, tgi, rmotion, nbh, true);
                if (gsc == REACHED) {
                    newSolution = true;
                    startMotion = startTree ? tgi.xmotion : addedMotion;
                    goalMotion = startTree ? addedMotion : tgi.xmotion;
                    // it must be the case that either the start tree or the goal tree has made some progress
                    // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                    // on the solution path
                    if (startMotion->parent != nullptr)
                        startMotion = startMotion->parent;
                    else
                        goalMotion = goalMotion->parent;
                }
                numBatchSamples += static_cast<int>(tStart_->size() + tGoal_->size()) - totalSamples;
            }


            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (newSolution && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {

                constructSolution(startMotion, goalMotion);
                solved = true;
                if (!optimize_ || upperTimeBound_ - minimumTime_ <= epsilon_) break; // first solution is enough or optimal solution is found
                // continue to look for solutions with the narrower time bound until the termination condition is met
            }
            else
            {
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (!startTree)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<og::PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return ob::PlannerStatus::APPROXIMATE_SOLUTION;
    }
    if (solved) {
        pdef_->addSolutionPath(bestSolution_, false, 0.0, getName());
    }

    return solved ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}

SpaceTimeRRT::GrowState SpaceTimeRRT::growTree(SpaceTimeRRT::TreeData &tree, SpaceTimeRRT::TreeGrowingInfo &tgi,
                                               SpaceTimeRRT::Motion *rmotion, std::vector<Motion *> &nbh, bool connect) {

    // If connect, advance from single nearest neighbor until the random state is reached or trapped
    if (connect) {
        GrowState gsc = ADVANCED;
        while (gsc == ADVANCED) {
            // get nearest motion
            Motion * nmotion = tree->nearest(rmotion);
            gsc = growTreeSingle(tree, tgi, rmotion, nmotion);
        }
        return gsc;
    }
    if (rewireState_ == OFF) {
        Motion * nmotion = tree->nearest(rmotion);
        return growTreeSingle(tree, tgi, rmotion, nmotion);
    }
    // get Neighborhood of random state
    getNeighbors(tree, rmotion, nbh);
    // in start tree sort by distance
    if (tgi.start) {
        std::sort(nbh.begin(), nbh.end(), [this, &rmotion] (Motion *a, Motion *b)
        {
            return si_->distance(a->state, rmotion->state) < si_->distance(b->state, rmotion->state);
        });
    }
    // in goal tree sort by time of root node
    else {
        std::sort(nbh.begin(), nbh.end(), [] (Motion *a, Motion *b)
        {
            auto t1 = a->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
            auto t2 = b->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
            return t1 < t2;
        });
    }

    // attempt to grow the tree for all neighbors in sorted order
    GrowState gs = TRAPPED;
    auto rt = rmotion->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    for (Motion *nmotion : nbh) {
        auto nt = nmotion->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        // trees grow only in one direction in time
        if ((tgi.start && nt > rt) || (!tgi.start && nt < rt)) continue;
        gs = growTreeSingle(tree, tgi, rmotion, nmotion);
        if (gs != TRAPPED)
            return gs;
    }
    // when radius_ is used for neighborhood calculation, the neighborhood might be empty
    if (nbh.empty()) {
        Motion * nmotion = tree->nearest(rmotion);
        return growTreeSingle(tree, tgi, rmotion, nmotion);
    }
    // can't grow Tree
    return gs;
}

SpaceTimeRRT::GrowState SpaceTimeRRT::growTreeSingle(SpaceTimeRRT::TreeData &tree, SpaceTimeRRT::TreeGrowingInfo &tgi,
                                                     SpaceTimeRRT::Motion *rmotion, SpaceTimeRRT::Motion *nmotion) {

    double rx = rmotion->state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double rt = rmotion->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    double nx = nmotion->state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double nt = nmotion->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    ob::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);

    if (d - epsilon_ > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
            return TRAPPED;

        /* Check if the interpolated state can be reached from the random state with respect to the max speed constraint.
         * When nmotion and rmotion are very close to the speed limit, interpolate can return an unreachable state.
         * In that case, increase the time difference by epsilon. */
        if (si_->distance(tgi.xstate, rmotion->state) > std::numeric_limits<double>::max()) {
            if (tgi.xstate->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position <
                    rmotion->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position) {
                tgi.xstate->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -= epsilon_;
            } else {
                tgi.xstate->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position += epsilon_;
            }
        }

        dstate = tgi.xstate;
        reach = false;
    }

    double dx = dstate->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double dt = dstate->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                       si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (!validMotion)
        return TRAPPED;

    auto *motion = new Motion(si_);
    si_->copyState(motion->state, dstate);
    motion->parent = nmotion;
    motion->root = nmotion->root;
    motion->parent->children.push_back(motion);
    tree->add(motion);
    tgi.xmotion = motion;

    return reach ? REACHED : ADVANCED;
}

void SpaceTimeRRT::constructSolution(SpaceTimeRRT::Motion *startMotion, SpaceTimeRRT::Motion *goalMotion) {

    if (goalMotion->connectionPoint == nullptr) {
        goalMotion->connectionPoint = startMotion;
        Motion *tMotion = goalMotion;
        while (tMotion != nullptr) {
            tMotion->numConnections++;
            tMotion = tMotion->parent;
        }
    }
    // check whether the found solution is an improvement
    auto newTime = goalMotion->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    if (newTime >= upperTimeBound_ - epsilon_)
        return;

    numSolutions++;
    isTimeBounded_ = true;
    if (!newBatchGoalMotions_.empty()) {
        goalMotions_.insert(goalMotions_.end(), newBatchGoalMotions_.begin(), newBatchGoalMotions_.end());
        newBatchGoalMotions_.clear();
    }

    /* construct the solution path */
    Motion *solution = startMotion;
    std::vector<Motion *> mpath1;
    while (solution != nullptr)
    {
        mpath1.push_back(solution);
        solution = solution->parent;
    }

    solution = goalMotion;
    std::vector<Motion *> mpath2;
    while (solution != nullptr)
    {
        mpath2.push_back(solution);
        solution = solution->parent;
    }

    auto path(std::make_shared<og::PathGeometric>(si_));
    path->getStates().reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i) {
        path->append(mpath1[i]->state);
    }
    for (auto &i : mpath2) {
        path->append(i->state);
    }


//     write to csv for visualization
    writeSamplesToCSV("prePruning");

    bestSolution_ = path;
    auto reachedGaol = path->getState(path->getStateCount() - 1);
    auto bestTime = reachedGaol->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    // Update Time Limit
    upperTimeBound_ = (bestTime - minimumTime_ ) * optimumApproxFactor_ + minimumTime_;
    // Prune Start and Goal Trees
    pruneStartTree();
    Motion* newSolution = pruneGoalTree();

    std::cout << "\nFound Solution for t = " << bestTime << ". New time bound = " << upperTimeBound_;

    writeSolutionToCSV();
    writeSamplesToCSV("afterPruning");

    // loop as long as a new solution is found by rewiring the goal tree
    if (newSolution != nullptr)
        constructSolution(newSolution->connectionPoint, goalMotion);
}

void SpaceTimeRRT::pruneStartTree() {

    std::queue<Motion *> queue;

    tStart_->clear();
    tStart_->add(startMotion_);
    for (auto &c : startMotion_->children) queue.push(c);
    while (!queue.empty()) {
        double t = queue.front()->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        double timeToNearestGoal = std::numeric_limits<double>::infinity();
        for (const auto &g : goalMotions_) {
            double deltaT = si_->getStateSpace()->as<space_time::AnimationStateSpace>()
                    ->timeToCoverDistance(queue.front()->state, g->state);
            if (deltaT < timeToNearestGoal) timeToNearestGoal = deltaT;
        }
        // Motion is still valid, re-add to tree
        if (t + timeToNearestGoal <= upperTimeBound_) {
            tStart_->add(queue.front());
            for (auto &c : queue.front()->children) queue.push(c);
        }
        // Motion is invalid due to the new time limit, delete motion
        else {
            // Remove the motion from its parent
            removeFromParent(queue.front());

            // for deletion first construct list of all descendants
            std::queue<Motion *> deletionQueue;
            std::vector<Motion *> deletionList;

            deletionQueue.push(queue.front());
            while (!deletionQueue.empty()) {
                for (auto &c : deletionQueue.front()->children)
                    deletionQueue.push(c);
                deletionList.push_back(deletionQueue.front());
                deletionQueue.pop();
            }

            // then free all descendants
            for (auto &m : deletionList) {
                // Erase the actual motion
                // First free the state
                if (m->state)
                    si_->freeState(m->state);
                // then delete the pointer
                delete m;
            }
        }
        // finally remove motion from the queue
        queue.pop();
    }
}

SpaceTimeRRT::Motion* SpaceTimeRRT::pruneGoalTree() {

    // it's possible to get multiple new solutions during the rewiring process. Store the best.
    double bestSolutionTime = upperTimeBound_;
    Motion * solutionMotion{nullptr};

    tGoal_->clear();
    std::vector<Motion *> validGoals;
    std::vector<Motion *> invalidGoals;

    // re-add goals with smallest time first
    std::sort(goalMotions_.begin(), goalMotions_.end(), [] (Motion * a, Motion * b) {
        return a->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position <
               b->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    });
    for (auto &m : goalMotions_) {
        double t = m->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        // add goal with all descendants to the tree
        if (t <= upperTimeBound_) {
            tGoal_->add(m);
            addDescendants(m, tGoal_);
            validGoals.push_back(m);
        }
        // try to rewire descendants to a valid goal
        else {
            invalidGoals.push_back(m);
            std::queue<Motion *> queue;
            for (auto &c : m->children)
                queue.push(c);
            while (!queue.empty()) {
                bool addedToTree = false;
                if (tGoal_->size() != 0) {
                    double costToGo = std::numeric_limits<double>::infinity();
                    double costSoFar = queue.front()->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
                    for (auto &g : validGoals) {
                        auto deltaT = si_->getStateSpace()->as<space_time::AnimationStateSpace>()
                                ->timeToCoverDistance(queue.front()->state, g->state);
                        if (deltaT < costToGo) costToGo = deltaT;
                    }
                    // try to rewire to the nearest neighbor

                    if (costSoFar + costToGo <= upperTimeBound_) {
                        TreeGrowingInfo tgi{};
                        tgi.xstate = si_->allocState();
                        tgi.start = false;
                        std::vector<Motion*> nbh;
                        GrowState gsc = growTree(tGoal_, tgi, queue.front(), nbh, true);
                        // connection successful, add all descendants and check if a new solution was found.
                        if (gsc == REACHED) {
                            // the motion was copied and added to the tree with a new parent
                            // adjust children and parent pointers
                            tgi.xmotion->children = queue.front()->children;
                            for (auto &c : tgi.xmotion->children) {
                                c->parent = tgi.xmotion;
                            }
                            tgi.xmotion->connectionPoint = queue.front()->connectionPoint;
                            tgi.xmotion->numConnections = queue.front()->numConnections;
                            Motion* p = tgi.xmotion->parent;
                            while (p != nullptr) {
                                p->numConnections += tgi.xmotion->numConnections;
                                p = p->parent;
                            }
                            addDescendants(tgi.xmotion, tGoal_);
                            // new solution found
                            if (tgi.xmotion->numConnections > 0 &&
                                tgi.xmotion->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position < bestSolutionTime)
                            {
                                bestSolutionTime = tgi.xmotion->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
                                std::queue<Motion*> connectionQueue;
                                connectionQueue.push(tgi.xmotion);
                                while (!connectionQueue.empty()) {
                                    if (connectionQueue.front()->connectionPoint != nullptr) {
                                        solutionMotion = connectionQueue.front();
                                        break;
                                    } else {
                                        for (Motion* c : connectionQueue.front()->children)
                                            connectionQueue.push(c);
                                    }
                                    connectionQueue.pop();
                                }
                            }
                            addedToTree = true;
                        }
                    }
                }
                // Free motion and state
                if (!addedToTree) {
                    // add children to queue, so they might be rewired
                    for (auto &c : queue.front()->children)
                        queue.push(c);
                }
                // Erase the actual motion
                // First free the state
                if (queue.front()->state)
                    si_->freeState(queue.front()->state);
                // then delete the pointer
                delete queue.front();

                queue.pop();
            }
        }
    }

    // remove invalid goals
    for (auto &g : invalidGoals) {
        for (auto it = goalMotions_.begin(); it != goalMotions_.end(); ++it) {
            if (*it == g) {
                goalMotions_.erase(it);
                break;
            }
        }
        if (g->state)
            si_->freeState(g->state);
        delete g;
    }

    return solutionMotion;
}

void SpaceTimeRRT::clear() {
    Planner::clear();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

void SpaceTimeRRT::getPlannerData(ob::PlannerData &data) const {

    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(ob::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(ob::PlannerDataVertex(motion->parent->state, 1), ob::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(ob::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(ob::PlannerDataVertex(motion->state, 2), ob::PlannerDataVertex(motion->parent->state, 2));
        }
        // add edges connecting the two trees
        if (motion->connectionPoint != nullptr)
            data.addEdge(data.vertexIndex(motion->connectionPoint->state), data.vertexIndex(motion->state));
    }

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}

void SpaceTimeRRT::removeFromParent(SpaceTimeRRT::Motion *m) {
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

/**
 * Adds all descendants of a motion to a given tree.
 *
 * @param m The motion, which descendants are added
 * @param tree The tree that the motions are added to

 */
void SpaceTimeRRT::addDescendants(SpaceTimeRRT::Motion *m, const SpaceTimeRRT::TreeData &tree) {

    std::queue<Motion *> queue;
    for (auto &c : m->children)
        queue.push(c);
    while (!queue.empty()) {
        for (auto &c : queue.front()->children)
            queue.push(c);
        queue.front()->root = m->root;
        tree->add(queue.front());
        queue.pop();
    }
}

void SpaceTimeRRT::getNeighbors(SpaceTimeRRT::TreeData &tree, SpaceTimeRRT::Motion *motion, std::vector<Motion *> &nbh) const {

    auto card = static_cast<double>(tree->size() + 1u);
    if (rewireState_ == RADIUS) {
        // r = min( r_rrt * (log(card(V))/card(V))^(1 / d + 1), distance)
        // for the formula change of the RRTStar paper, see 'Revisiting the asymptotic optimality of RRT*'
        double r = std::min(maxDistance_, r_rrt_ * std::pow(log(card) / card, 1.0 / 1.0 + static_cast<double>(si_->getStateDimension())));
    }
    else if (rewireState_ == KNEAREST){
        // k = k_rrt * log(card(V))
        unsigned int k = std::ceil(k_rrt_ * log(card));
        tree->nearestK(motion, k, nbh);
    }
}

bool SpaceTimeRRT::rewireGoalTree(SpaceTimeRRT::Motion *addedMotion) {

    bool solved = false;
    std::vector<Motion*> nbh;
    getNeighbors(tGoal_, addedMotion, nbh);
    double nodeT = addedMotion->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    double goalT = addedMotion->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    for (Motion* otherMotion : nbh) {
        double otherNodeT = otherMotion->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        double otherGoalT = otherMotion->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        // rewire, if goal time is improved and the otherMotion node can be connected to the added node
        if (otherNodeT < nodeT && goalT < otherGoalT && si_->checkMotion(otherMotion->state, addedMotion->state)) {
//            auto otherX = otherMotion->state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
//            auto oldRootX = otherMotion->root->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
//            auto newRootX = addedMotion->root->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
//            std::cout << "\n Rewiring: \nNode [" << otherX << ", " << otherNodeT << "] \n with old root ["
//                << oldRootX << ", " << otherGoalT << "] \n to new root ["
//                << newRootX << ", " << goalT << "]\n";
            // decrease connection count of old ancestors
            if (otherMotion->numConnections > 0) {
                Motion * p = otherMotion->parent;
                while (p != nullptr) {
                    p->numConnections--;
                    p = p->parent;
                }
            }
            removeFromParent(otherMotion);
            otherMotion->parent = addedMotion;
            otherMotion->root = addedMotion->root;
            addedMotion->children.push_back(otherMotion);
            // increase connection count of new ancestors
            if (otherMotion->numConnections > 0) {
                Motion * p = otherMotion->parent;
                while (p != nullptr) {
                    p->numConnections++;
                    p = p->parent;
                }
                if (otherMotion->root->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position < upperTimeBound_) {
                    solved = true;
                }
            }
        }
    }

    return solved;
}

void SpaceTimeRRT::calculateRewiringLowerBounds() {

    const auto dim = static_cast<double>(si_->getStateDimension());

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // prunedMeasure_ is set to si_->getSpaceMeasure();
    r_rrt_ =
            rewireFactor_ * std::pow(2 * (1.0 + 1.0 / dim) *
            (si_->getSpaceMeasure() / ompl::unitNBallMeasure(si_->getStateDimension())), 1.0 / dim);

    // k_rrg > e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * boost::math::constants::e<double>() * (1.0 + 1.0 / dim);
}

bool SpaceTimeRRT::sampleGoalTime(ob::State * goal) {
    double ltb, utb;
    double minTime = si_->getStateSpace()->as<space_time::AnimationStateSpace>()
            ->timeToCoverDistance(startMotion_->state, goal);
    if (isTimeBounded_) {
        ltb = minTime;
        utb = upperTimeBound_;
    }
    else if (sampleOldBatch_) {
        ltb = minTime;
        utb = minTime * oldBatchTimeBoundFactor_;
    }
    else {
        ltb = minTime * oldBatchTimeBoundFactor_;
        utb = minTime * newBatchTimeBoundFactor_;
    }

    if (ltb > utb) return false; // goal can't be reached in time

    double time = ltb == utb ? ltb : rng_.uniformReal(ltb, utb);
    goal->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = time;
    return true;
}

ob::State *SpaceTimeRRT::nextGoal(int n) {
    static ob::PlannerTerminationCondition ptc = ob::plannerNonTerminatingCondition();
    return nextGoal(ptc, n);
}

ob::State *SpaceTimeRRT::nextGoal(const ob::PlannerTerminationCondition &ptc) {
    return nextGoal(ptc, -1);
}

ob::State *SpaceTimeRRT::nextGoal(const ob::PlannerTerminationCondition &ptc, int n) {
    if (pdef_->getGoal() != nullptr)
    {
        const ob::GoalSampleableRegion *goal =
                pdef_->getGoal()->hasType(ob::GOAL_SAMPLEABLE_REGION) ? pdef_->getGoal()->as<ob::GoalSampleableRegion>() : nullptr;

        if (goal != nullptr)
        {
            if (tempState_ == nullptr)
                tempState_ = si_->allocState();
            int tryCount = 0;
            do
            {
                goal->sampleGoal(tempState_); // sample space component
                bool inTime = sampleGoalTime(tempState_); // sample time component dependant on sampled space
                bool bounds = inTime && si_->satisfiesBounds(tempState_);
                bool valid = bounds && si_->isValid(tempState_);
                if (valid)
                {
                    return tempState_;
                }
            } while (!ptc.eval() && ++tryCount != n);
        }
    }

    return nullptr;
}

void SpaceTimeRRT::writeSamplesToCSV(const std::string& type) {
    std::ofstream outfile ("data/debug/" + std::to_string(numSolutions) + type + ".csv");
    std::string delim = ",";

    ob::PlannerData data(si_);
    getPlannerData(data);

    outfile << "x" << delim << "time" << delim << "incoming edge" << delim << "start or goal\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
//        double y = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        int startOrGoalTag = data.getVertex(i).getTag();


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        std::stringstream ssIn;
        for (auto edgeIndex : inEdgeIndexes) {
            ssIn << "#" << edgeIndex;
        }
        std::string inEdges = inEdgeIndexes.empty()? "#" : ssIn.str();
//        std::string startOrGoal = startOrGoalTag == 1 ? "start" : "goal";

        // write node data to csv
        outfile << x << delim << t << delim << inEdges << delim << startOrGoalTag << "\n";

    }

    outfile.close();
}

void SpaceTimeRRT::writeSolutionToCSV() {
    std::ofstream outfile ("data/debug/" + std::to_string(numSolutions) + "path.csv");
    std::string delim = ",";

    outfile << "x" << delim << "time" << delim << "upper time bound\n";

    bool first = true;
    auto path = bestSolution_->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        if (first) {
            outfile << x << delim << t << delim << upperTimeBound_ << "\n";
        }
        else {
            outfile << x << delim << t << "\n";
        }
        first = false;
    }

    outfile.close();
}
}