//
// Created by francesco on 12.08.21.
//

#include "Benchmarker.h"

Benchmarker::DataPoint::DataPoint(int iteration, std::vector<std::pair<double, double>> solutions) : iteration(
        iteration), solutions(std::move(solutions)) {}


void Benchmarker::run() {
//    for (int i = 0; i < iterations; ++i) {
//        nd::TimeNDPlanner planner{dimensions_};
//        planner.loadConfigFromJSON(testset_ + ".json");
//        planner.setPlanner(plannerType_);
//        planner.setSolveTime(solveTime_);
//        planner.planMotion();
//        dataset_.emplace_back(i, planner.getSolutions());
//    }
//
//    WriteData();
}

void Benchmarker::WriteData() {
    std::ostringstream oss;
    oss << "data/benchmarks/" << dimensions_ << "/" << testset_;
    std::ofstream outfile (oss.str());

    std::string delim = ",";

    outfile << "it" << delim << "runtime" << delim << "cost\n";
    for (const auto &data : dataset_) {
        for (const auto &pair : data.solutions) {
            outfile << data.iteration << delim << pair.first << delim << pair.second << "\n";
        }
    }

    outfile.close();
}
