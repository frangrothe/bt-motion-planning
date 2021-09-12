
//
// Created by francesco on 25.07.21.
//

#include "TimeNDPlanner.h"

namespace nd {

TimeNDPlanner::TimeNDPlanner(int d) : startPosition_(d_, 0.0), goalPosition_(d_, 1.0), d_(d) {}

og::SimpleSetup TimeNDPlanner::createSimpleSetup() {
    double distance = sqrt(d_); // distance between start (0, ..., 0) and goal (1, ..., 1)
    double speed = distance;
    auto vectorSpace = std::make_shared<ob::RealVectorStateSpace>(d_);
    auto space = std::make_shared<space_time::AnimationStateSpace>(vectorSpace, speed, timeWeight_);

    // Set the bounds for R1
    ob::RealVectorBounds bounds(d_);
    bounds.setLow(-2.0);
    bounds.setHigh(2.0);
    vectorSpace->setBounds(bounds);
    // set time bound if initialized
    if (upperTimeBound_ > 0.0)
        space->setTimeBounds(0.0, upperTimeBound_);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
//    si->setStateValidityChecker(std::make_shared<TimeNDStateValidityChecker>(si, d_, constraints_, bounds, agentRadius_));
    si->setStateValidityChecker(std::make_shared<TimeNDNarrowPassageValidityChecker>(si, bounds));
    si->setMotionValidator(std::make_shared<TimeNDMotionValidator>(si, d_, speed));
    og::SimpleSetup ss(si);

    // Start and goal
    ob::ScopedState<> start(space);
    for (int i = 0; i < startPosition_.size(); ++i) {
        start[i] = startPosition_[i];
    }

    ob::ScopedState<> goal(space);
    for (int i = 0; i < goalPosition_.size(); ++i) {
        goal[i] = goalPosition_[i];
    }

    auto goalRegion = std::make_shared<TimeNDGoal>(si, d_);
    goalRegion->setState(goal);
    if (upperTimeBound_ > 0.0)
        goalRegion->setUpperTimeBound(upperTimeBound_);

    ss.setStartState(start);
    ss.setGoal(goalRegion);

    ss.setOptimizationObjective(std::make_shared<space_time::MinimizeArrivalTime>(ss.getSpaceInformation()));

    return ss;
}

void TimeNDPlanner::plan() {

    auto ss = createSimpleSetup();
    // Set Planner
    switch (plannerType_) {
        case SpaceTimeRRT:
            ss.setPlanner(createSpaceTimeRRT(ss.getSpaceInformation())); break;
        case RRTConnect:
            ss.setPlanner(createRRTConnect(ss.getSpaceInformation())); break;
        case RRTStar:
            ss.setPlanner(createRRTStar(ss.getSpaceInformation())); break;
        case LBTRRT:
            ss.setPlanner(createLBTRRT(ss.getSpaceInformation())); break;
        case LazyLBTRRT:
            ss.setPlanner(createLazyLBTRRT(ss.getSpaceInformation())); break;
        case TRRT:
            ss.setPlanner(createTRRT(ss.getSpaceInformation())); break;
        case BiTRRT:
            ss.setPlanner(createBiTRRT(ss.getSpaceInformation())); break;
        case PRMStar:
            ss.setPlanner(ob::PlannerPtr(new og::PRMstar(ss.getSpaceInformation()))); break;
        case LazyPRMStar:
            ss.setPlanner(createLazyPRMStar(ss.getSpaceInformation())); break;
    }
    // Planner executes
    ob::PlannerStatus solved = ss.solve(solveTime_);
    if (solved) {
        ob::PlannerData data(ss.getSpaceInformation());
        ss.getPlannerData(data);

        std::cout << "\nFound solution:" << std::endl;

        // print the path to screen
        ob::PathPtr path = std::make_shared<og::PathGeometric>(ss.getSolutionPath());
        path->print(std::cout);

        std::cout << "\nNumber of Samples: " << data.numVertices();
        std::cout << "\nExact Solution: " << (ss.haveExactSolutionPath()? "yes" : "no");
        std::cout << "\nApproximate Solution: " << (ss.haveSolutionPath() && !ss.haveExactSolutionPath()? "yes" : "no") << std::endl;

//        writeSolutionToJSON(path);

    }
}

void TimeNDPlanner::benchmark() {
    auto ss = createSimpleSetup();

    // First we create a benchmark class:
    ompl::tools::Benchmark b(ss, "my experiment");

    // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
//    b.addExperimentParameter("num_dofs", "INTEGER", "6");
//    b.addExperimentParameter("num_obstacles", "INTEGER", "10");

    // We add the planners to evaluate.
//    b.addPlanner(ob::PlannerPtr(new og::PRMstar(ss.getSpaceInformation())));

    // For planners that we want to configure in specific ways,
    // the ompl::base::PlannerAllocator should be used:
    if (plannerType_ == SpaceTimeRRT) {
        b.addPlannerAllocator(std::bind(&TimeNDPlanner::createSpaceTimeRRT, this, std::placeholders::_1));
        b.setPostRunEvent(std::bind(&TimeNDPlanner::RecordTimeSpaceTimeRRT, this, std::placeholders::_1, std::placeholders::_2));
    }
    else {
        b.addPlannerAllocator(std::bind(&TimeNDPlanner::createRRTConnect, this, std::placeholders::_1));
        b.addPlannerAllocator(std::bind(&TimeNDPlanner::createRRTStar, this, std::placeholders::_1));
        b.setPostRunEvent(std::bind(&TimeNDPlanner::RecordBestCost, this, std::placeholders::_1, std::placeholders::_2));
    }

    // Now we can benchmark: 5 second time limit for each plan computation,
    // 100 MB maximum memory usage per plan computation, 50 runs for each planner
    // and true means that a text-mode progress bar should be displayed while
    // computation is running.
    ompl::tools::Benchmark::Request req;
    req.maxTime = solveTime_;
    req.maxMem = 4000.0;
    req.runCount = 100;
    req.displayProgress = true;
    req.timeBetweenUpdates = 0.002; // in seconds
    b.benchmark(req);

    // This will generate a file of the form ompl_host_time.log
    std::ostringstream oss;
    std::string s = plannerType_ == SpaceTimeRRT ? "spacetime" : std::to_string(int(upperTimeBound_));
    oss << "data/benchmarks/narrow" << d_ << "/n8_" << s << ".log";
    b.saveResultsToFile(oss.str().c_str());
}

void TimeNDPlanner::loadConfigFromJSON(const std::string& filename) {
    filename_ = filename;
    // read a JSON file
    std::ostringstream oss;
    oss << "data/testsets/" << d_ << "/" << filename;
    std::cout << oss.str() << std::endl;
    std::ifstream ifs(oss.str());
    nlohmann::json j = nlohmann::json::parse(ifs);

    std::vector<double> start = j["start"];
    startPosition_ = start;
    std::vector<double> goal = j["goal"];
    goalPosition_ = goal;
    double radius = j["radius"];
    agentRadius_ = radius;

    for (auto & o : j["obstacles"]) {
        double r = o["r"];
        std::vector<std::vector<double>> path = o["path"];
        constraints_.emplace_back(r, path);
    }
}

void TimeNDPlanner::writeSolutionToJSON(const ompl::base::PathPtr &pathPtr) {

    std::vector<std::vector<double>> solution;
    nlohmann::json fullJson;
    auto path = pathPtr->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double *values = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        std::vector<double> position(d_);
        for (int i = 0; i < d_; ++i) {
            position[i] = values[i];
        }
        nlohmann::json j;
        j["time"] = t;
        j["pos"] = position;
        fullJson.push_back(j);
    }

    std::ostringstream oss;
    oss << "data/testsets/" << d_ << "/" << "solution_" << filename_;
    // write prettified JSON to another file
    std::ofstream outfile(oss.str());
    outfile << std::setw(4) << fullJson << std::endl;
}

ob::PlannerPtr TimeNDPlanner::createRRTConnect(const ob::SpaceInformationPtr &si) {
    auto *rrtConnect = new og::RRTConnect(si);
    rrtConnect->setRange(plannerRangeFactor_ * sqrt(d_));
    return ob::PlannerPtr(rrtConnect);
}

ompl::base::PlannerPtr TimeNDPlanner::createRRTStar(const ompl::base::SpaceInformationPtr &si) {
    auto *rrtStar = new og::RRTstar(si);
    rrtStar->setRange(plannerRangeFactor_ * sqrt(d_));
    return ob::PlannerPtr(rrtStar);
}

ompl::base::PlannerPtr TimeNDPlanner::createSpaceTimeRRT(const ompl::base::SpaceInformationPtr &si) {
    auto *spaceTimeRRT = new space_time::SpaceTimeRRT(si);
    spaceTimeRRT->setRange(plannerRangeFactor_ * sqrt(d_));
    spaceTimeRRT->setBatchSize(batchSize_);
    return ob::PlannerPtr (spaceTimeRRT);
}

ompl::base::PlannerPtr TimeNDPlanner::createLBTRRT(const ompl::base::SpaceInformationPtr &si) {
    auto *lbtrrt = new og::LBTRRT(si);
    lbtrrt->setRange(plannerRangeFactor_ * sqrt(d_));
    return ob::PlannerPtr(lbtrrt);
}

ompl::base::PlannerPtr TimeNDPlanner::createLazyLBTRRT(const ompl::base::SpaceInformationPtr &si) {
    auto *lazylbtrrt = new og::LazyLBTRRT(si);
    lazylbtrrt->setRange(plannerRangeFactor_ * sqrt(d_));
    return ob::PlannerPtr(lazylbtrrt);
}

ompl::base::PlannerPtr TimeNDPlanner::createTRRT(const ompl::base::SpaceInformationPtr &si) {
    auto *trrt = new og::TRRT(si);
    trrt->setRange(plannerRangeFactor_ * sqrt(d_));
    return ob::PlannerPtr(trrt);
}

ompl::base::PlannerPtr TimeNDPlanner::createBiTRRT(const ompl::base::SpaceInformationPtr &si) {
    auto *bitrrt = new og::BiTRRT(si);
    bitrrt->setRange(plannerRangeFactor_ * sqrt(d_));
    return ob::PlannerPtr(bitrrt);
}

ompl::base::PlannerPtr TimeNDPlanner::createLazyPRMStar(const ompl::base::SpaceInformationPtr &si) {
    auto *lazyprmstar = new og::LazyPRMstar(si);
    lazyprmstar->setRange(plannerRangeFactor_ * sqrt(d_));
    return ob::PlannerPtr(lazyprmstar);
}

void TimeNDPlanner::RecordTimeSpaceTimeRRT(const ob::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run) {
    double timeToFirstSolution = planner->as<space_time::SpaceTimeRRT>()->getTimeToFirstSolution();
    run["time_first_solution REAL"] = std::to_string(timeToFirstSolution);
}

void TimeNDPlanner::RecordBestCost(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run) {
    auto path = planner->getProblemDefinition()->getSolutionPath()->as<og::PathGeometric>();
    auto lastState = path->getState(path->getStateCount() - 1);
    double t = lastState->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    run["cost REAL"] = std::to_string(t);

}

//void TimeNDPlanner::RecordTimeRRTS(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run) {
//    double timeToFirstSolution = planner->as<og::RRTstar>()->getTimeToFirstSolution();
//    run["time_first_solution REAL"] = std::to_string(timeToFirstSolution);
//}

}
