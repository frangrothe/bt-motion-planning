//
// Created by francesco on 18.06.21.
//


#include <ompl/tools/config/SelfConfig.h>
#include <fstream>
#include "SpaceTimeRRT.h"

namespace space_time {


SpaceTimeRRT::SpaceTimeRRT(const ompl::base::SpaceInformationPtr &si) : Planner(si, "SpaceTimeRRT"),
                                                                        sampler_(&(*si)), goalSet_(&goalSetCmp){

    Planner::declareParam<double>("range", this, &SpaceTimeRRT::setRange, &SpaceTimeRRT::getRange, "0.:1.:10000.");
    connectionPoint_ = std::make_pair<ob::State *, ob::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

SpaceTimeRRT::~SpaceTimeRRT() {
    freeMemory();
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

    sampler_.clear();

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


        auto s = si_->allocState();
        si_->copyState(s, st);
        sampler_.addStartState(s);
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

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    ob::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    while (!ptc)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || goalSet_.size() < tGoal_->size() / 4)
        {
            const ob::State *st = tGoal_->size() == 0 ? nextGoal(ptc) : nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
                goalSet_.insert(motion);

                auto s = si_->allocState();
                si_->copyState(s, st);
                sampler_.addGoalState(s);

                minimumTime_ = std::min(minimumTime_, si_->getStateSpace()
                ->as<space_time::AnimationStateSpace>()->timeToCoverDistance(startMotion_->state, st));
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_.sample(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);
        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;

            while (gsc == ADVANCED) gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;
                constructSolution(startMotion, goalMotion);
                solved = true;
                if (upperTimeBound_ - minimumTime_ <= epsilon_) break; // optimal solution is found
                break;
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
                                               SpaceTimeRRT::Motion *rmotion) {
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    ob::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);

    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
            return TRAPPED;

        dstate = tgi.xstate;
        reach = false;
    }

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

    connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

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
    for (int i = mpath1.size() - 1; i >= 0; --i)
        path->append(mpath1[i]->state);
    for (auto &i : mpath2)
        path->append(i->state);

    bestSolution_ = path;
    auto reachedGaol = path->getState(path->getStateCount() - 1);
    auto bestTime = reachedGaol->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    // Update Time Limit
    upperTimeBound_ = (bestTime - minimumTime_ ) * solutionImproveFactor_ + minimumTime_;
    // Prune Start and Goal Trees
    int numStartPruned, numGoalPruned;
    bool solvedAgain = false;
    numStartPruned = pruneStartTree();
    std::tie(numGoalPruned, solvedAgain) = pruneGoalTree(goalMotion);

    std::cout << "\n" << numStartPruned << " START STATES PRUNED";
    std::cout << "\n" << numGoalPruned << " GOAL STATES PRUNED";
    // loop as long as a new solution is found by rewiring the goal tree
    if (solvedAgain) constructSolution(startMotion, goalMotion);
}

int SpaceTimeRRT::pruneStartTree() {

    int numPruned = 0;
    std::queue<Motion *> queue;

    tStart_->clear();
    tStart_->add(startMotion_);
    for (auto &c : startMotion_->children) queue.push(c);
    while (!queue.empty()) {
        double t = queue.front()->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        double timeToNearestGoal = std::numeric_limits<double>::infinity();
        for (const auto &g : goalSet_) {
            double deltaT = si_->getStateSpace()->template as<space_time::AnimationStateSpace>()
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
                numPruned++;
            }
        }
        // finally remove motion from the queue
        queue.pop();
    }

    return numPruned;
}

std::tuple<int, bool> SpaceTimeRRT::pruneGoalTree(Motion * goalMotion) {
    bool solvedAgain = false; // it's possible to get a new solution during the rewiring process
    int numPruned = 0;
    tGoal_->clear();
    std::vector<Motion *> validGoals;
    std::vector<Motion *> invalidGoals;

    for (auto &m : goalSet_) {
        double t = m->state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        // add goal with all descendants to the tree
        if (t <= upperTimeBound_) {
            tGoal_->add(m);
            bool addedGoalMotion = false;
            addDescendants(m, tGoal_, goalMotion, &addedGoalMotion);
            if (addedGoalMotion || m == goalMotion) solvedAgain = true;
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
                        auto deltaT = si_->getStateSpace()->template as<space_time::AnimationStateSpace>()
                                ->timeToCoverDistance(queue.front()->state, g->state);
                        if (deltaT < costToGo) costToGo = deltaT;
                    }
                    // try to rewire to the nearest neighbor

                    if (costSoFar + costToGo <= upperTimeBound_) {
                        TreeGrowingInfo tgi;
                        tgi.xstate = si_->allocState();
                        tgi.start = false;
                        GrowState gsc = ADVANCED;
                        while (gsc == ADVANCED) gsc = growTree(tGoal_, tgi, queue.front());
                        // connection successful, add all descendants and check if the goal motion is added.
                        // In that case, the problem is solved again for an improved time
                        if (gsc == REACHED) {
                            bool addedGoalMotion = false;
                            addDescendants(queue.front(), tGoal_, goalMotion, &addedGoalMotion);
                            if (addedGoalMotion || queue.front() == goalMotion) solvedAgain = true;
                            addedToTree = true;
                        }
                    }
                }
                // Free motion and state
                if (!addedToTree) {
                    // add children to queue, so they might be rewired
                    for (auto &c : queue.front()->children)
                        queue.push(c);
                    // Erase the actual motion
                    // First free the state
                    if (queue.front()->state)
                        si_->freeState(queue.front()->state);
                    // then delete the pointer
                    delete queue.front();
                    numPruned++;
                }

                queue.pop();
            }
        }
    }

    // remove invalid goals
    for (auto &g : invalidGoals) {
        goalSet_.erase(g);
        if (g->state)
            si_->freeState(g->state);
        delete g;
        numPruned++;
    }

    return std::make_tuple(numPruned, solvedAgain);
}

void SpaceTimeRRT::clear() {
    Planner::clear();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<ob::State *, ob::State *>(nullptr, nullptr);
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
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
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

void SpaceTimeRRT::addDescendants(SpaceTimeRRT::Motion *m, const SpaceTimeRRT::TreeData &tree,
                                  SpaceTimeRRT::Motion *goalMotion, bool *addedGoalMotion) {
    std::queue<Motion *> queue;
    for (auto &c : m->children)
        queue.push(c);
    while (!queue.empty()) {
        for (auto &c : queue.front()->children)
            queue.push(c);
        tree->add(queue.front());
        if (queue.front() == goalMotion) *addedGoalMotion = true;
        queue.pop();
    }
}

void SpaceTimeRRT::writeSamplesToCSV(const std::string& num) {
    std::ofstream outfile ("data/debug/samples" + num + ".csv");
    std::string delim = ",";

    ob::PlannerData data(si_);
    getPlannerData(data);

    outfile << "x" << delim << "time" << delim << "incoming edge" << delim << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
//        double y = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        std::stringstream ssIn;
        for (auto edgeIndex : inEdgeIndexes) {
            ssIn << "#" << edgeIndex;
        }
        std::string inEdges = inEdgeIndexes.empty()? "#" : ssIn.str();

        // Get outgoing edges for node
        std::vector< unsigned int > outEdgeIndexes{};
        data.getEdges(i, outEdgeIndexes);
        std::stringstream ssOut;
        for (auto edgeIndex : outEdgeIndexes) {
            ssOut << "#" << edgeIndex;
        }
        std::string outEdges = outEdgeIndexes.empty()? "#" : ssOut.str();

        // write node data to csv
        outfile << x << delim << t << delim << inEdges << delim << outEdges << "\n";

    }

    outfile.close();
}

bool SpaceTimeRRT::sampleGoalTime(ob::State * goal) {
    double lowerTimeBound = si_->getStateSpace()->as<space_time::AnimationStateSpace>()
            ->timeToCoverDistance(startMotion_->state, goal);
    if (lowerTimeBound > upperTimeBound_) return false; // goal can't be reached in time

    double time = lowerTimeBound == upperTimeBound_ ? lowerTimeBound : rng_.uniformReal(lowerTimeBound, upperTimeBound_);
    goal->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = time;
    return true;
}

ob::State *SpaceTimeRRT::nextGoal() {
    static ob::PlannerTerminationCondition ptc = ob::plannerAlwaysTerminatingCondition();
    return nextGoal(ptc);
}

ob::State *SpaceTimeRRT::nextGoal(const ob::PlannerTerminationCondition &ptc) {
    if (pdef_->getGoal() != nullptr)
    {
        const ob::GoalSampleableRegion *goal =
                pdef_->getGoal()->hasType(ob::GOAL_SAMPLEABLE_REGION) ? pdef_->getGoal()->as<ob::GoalSampleableRegion>() : nullptr;

        if (goal != nullptr)
        {
            if (tempState_ == nullptr)
                tempState_ = si_->allocState();
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
            } while (!ptc);
        }
    }

    return nullptr;
}
}