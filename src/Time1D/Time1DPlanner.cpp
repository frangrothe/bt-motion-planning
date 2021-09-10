//
// Created by francesco on 19.05.21.
//


#include "Time1DPlanner.h"


namespace time_1d {

Time1DPlanner::Time1DPlanner(std::string filename) : filename_(std::move(filename)) {}

og::SimpleSetup Time1DPlanner::createSimpleSetup() {

    auto vectorSpace = std::make_shared<ob::RealVectorStateSpace>(1);
    auto space = std::make_shared<space_time::AnimationStateSpace>(vectorSpace, vMax_, timeWeight_);

    // Set the bounds for R1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(xBoundLow_);
    bounds.setHigh(xBoundHigh_);
    vectorSpace->setBounds(bounds);

//     Set the bounds for Time
    if (timeBoundHigh_ > timeBoundLow_) {
        space->setTimeBounds(timeBoundLow_, timeBoundHigh_);
    }

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<Time1DStateValidityChecker>(si, constraints_));
    si->setMotionValidator(std::make_shared<Time1DMotionValidator>(si, vMax_));
    og::SimpleSetup ss(si);

    ob::ScopedState<> start(space);
    start[0] = xStart_; // r1State
//    auto utb = timeBoundHigh_ > timeBoundLow_ ? timeBoundHigh_ : timeBoundLow_ + 1.0;
//    auto goal = std::make_shared<Time1DGoalRegion>(si, goalRegions_[0].first, goalRegions_[0].second, timeBoundLow_, utb);
    auto goal = std::make_shared<VectorSpaceGoalRegion>(si, goalRegions_);
    ss.setStartState(start);
    ss.setGoal(goal);

    ss.setOptimizationObjective(std::make_shared<space_time::MinimizeArrivalTime>(ss.getSpaceInformation()));

    return ss;
}

void Time1DPlanner::planMotion() {

   auto ss = createSimpleSetup();
    switch (plannerType_) {
        case SpaceTimeRRT:
            ss.setPlanner(createSpaceTimeRRT(ss.getSpaceInformation())); break;
        case RRTConnect:
            ss.setPlanner(createRRTConnect(ss.getSpaceInformation())); break;
        case RRTStar:
            ss.setPlanner(createRRTStar(ss.getSpaceInformation())); break;
    }

    //(4) Planner executes
    ob::PlannerStatus solved = ss.solve(solveTime_);

    if (solved)
    {
        ob::PlannerData data(ss.getSpaceInformation());
        ss.getPlannerData(data);

        std::cout << "\nFound solution:" << std::endl;

        // print the path to screen
        ob::PathPtr path = std::make_shared<og::PathGeometric>(ss.getSolutionPath());
        path->print(std::cout);

        std::cout << "\nNumber of Samples: " << data.numVertices();
        std::cout << "\nExact Solution: " << (ss.haveExactSolutionPath()? "yes" : "no");
        std::cout << "\nApproximate Solution: " << (ss.haveSolutionPath() && !ss.haveExactSolutionPath()? "yes" : "no") << std::endl;
//
        writeSamplesToCSV(data);
        writePathToCSV(path);
//        writeConstraintsToCSV();
//        writeGoalRegionToCSV();
    }
    else
        std::cout << "No solution found" << std::endl;

}

void Time1DPlanner::benchmark() {
    auto ss = createSimpleSetup();

    // First we create a benchmark class:
    ompl::tools::Benchmark b(ss, "my experiment");

    if (plannerType_ == SpaceTimeRRT) {
        b.addPlannerAllocator(std::bind(&Time1DPlanner::createSpaceTimeRRT, this, std::placeholders::_1));
        b.setPostRunEvent(std::bind(&Time1DPlanner::RecordTimeSpaceTimeRRT, this, std::placeholders::_1, std::placeholders::_2));
    }
    else {
        b.addPlannerAllocator(std::bind(&Time1DPlanner::createRRTConnect, this, std::placeholders::_1));
        b.addPlannerAllocator(std::bind(&Time1DPlanner::createRRTStar, this, std::placeholders::_1));
        b.setPostRunEvent(std::bind(&Time1DPlanner::RecordBestCost, this, std::placeholders::_1, std::placeholders::_2));
    }

    ompl::tools::Benchmark::Request req;
    req.maxTime = solveTime_;
    req.maxMem = 4000.0;
    req.runCount = 100;
    req.displayProgress = true;
    req.timeBetweenUpdates = 0.001; // in seconds
    b.benchmark(req);

    // This will generate a file of the form ompl_host_time.log
    std::ostringstream oss;
    std::string s = plannerType_ == SpaceTimeRRT ? "spacetime" : std::to_string(int(timeBoundHigh_));
    oss << "data/benchmarks/narrow" << "/n3_" << s << ".log";
    b.saveResultsToFile(oss.str().c_str());
}

void Time1DPlanner::writeSamplesToCSV(const ob::PlannerData &data) {

    std::ofstream outfile ("data/" + filename_ + "/final_samples.csv");

    outfile << "x" << delim_ << "time" << delim_ << "incoming edge" << delim_ << "outgoing edges" << delim_ << "tag\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        int tag = data.getVertex(i).getTag();


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
        outfile << x << delim_ << t << delim_ << inEdges << delim_ << outEdges << delim_ << tag << "\n";

    }

    outfile.close();
}

void Time1DPlanner::writePathToCSV(const ob::PathPtr &pathPointer) {
    std::ofstream outfile ("data/" + filename_ + "/final_path.csv");

    outfile << "x" << delim_ << "time\n";

    auto path = pathPointer->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        outfile << x << delim_ << t << "\n";
    }

    outfile.close();
}

void Time1DPlanner::writeConstraintsToCSV() {

    std::ofstream outfile ("data/" + filename_ + "/constraints.csv");

    outfile << "x lb" << delim_ << "x ub" << delim_ << "t lb" << delim_ << "t ub\n";
    for (auto c : constraints_) {
        outfile << c.x_lb << delim_ << c.x_ub << delim_ << c.t_lb << delim_ << c.t_ub << "\n";
    }

    outfile.close();
}

void Time1DPlanner::writeGoalRegionToCSV() {
    std::ofstream outfile ("data/" + filename_ + "/goal.csv");

    outfile << "x lb" << delim_ << "x ub" << delim_ << "t lb" << delim_ << "t ub\n";
    for (auto &g : goalRegions_) {
        double minTime = std::fabs((g.first - xStart_)) / vMax_; // minimum time at which the goal can be reached
        outfile << g.first << delim_ << g.second << delim_ << minTime << delim_ << timeBoundHigh_ << "\n";
    }


    outfile.close();
}

ompl::base::PlannerPtr Time1DPlanner::createRRTConnect(const ompl::base::SpaceInformationPtr &si) {
    auto *rrtConnect = new og::RRTConnect(si);
    rrtConnect->setRange(plannerRange_);
    return ob::PlannerPtr(rrtConnect);
}

ompl::base::PlannerPtr Time1DPlanner::createRRTStar(const ompl::base::SpaceInformationPtr &si) {
    auto *rrtStar = new og::RRTstar(si);
    rrtStar->setRange(plannerRange_);
    return ob::PlannerPtr(rrtStar);
}

ompl::base::PlannerPtr Time1DPlanner::createSpaceTimeRRT(const ompl::base::SpaceInformationPtr &si) {
    auto *spaceTimeRRT = new space_time::SpaceTimeRRT(si);
    spaceTimeRRT->setRange(plannerRange_);
    return ob::PlannerPtr (spaceTimeRRT);
}

void Time1DPlanner::RecordTimeSpaceTimeRRT(const ompl::base::PlannerPtr &planner,
                                           ompl::tools::Benchmark::RunProperties &run) {
    double timeToFirstSolution = planner->as<space_time::SpaceTimeRRT>()->getTimeToFirstSolution();
    run["time_first_solution REAL"] = std::to_string(timeToFirstSolution);

}

void Time1DPlanner::RecordBestCost(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run) {
    auto path = planner->getProblemDefinition()->getSolutionPath()->as<og::PathGeometric>();
    double t = std::numeric_limits<double>::infinity();
    if (path != nullptr) {
        auto lastState = path->getState(path->getStateCount() - 1);
        t = lastState->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    }

    run["cost REAL"] = std::to_string(t);
}
}

