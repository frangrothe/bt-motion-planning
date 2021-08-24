//
// Created by francesco on 12.08.21.
//

#ifndef BT_ROBOTICS_BENCHMARKER_H
#define BT_ROBOTICS_BENCHMARKER_H

#include "../TimeND/TimeNDPlanner.h"

class Benchmarker {

public:
    void run();

private:
    double solveTime_ = 20.0;
    int iterations = 10;
    int dimensions_ = 2;
    std::string testset_ = "a1";
    nd::TimeNDPlanner::Planners plannerType_ = nd::TimeNDPlanner::RRTConnect;

    struct DataPoint {
        DataPoint(int iteration, std::vector<std::pair<double, double>> solutions);

        int iteration;
        std::vector<std::pair<double, double>> solutions;
    };

    std::vector<DataPoint> dataset_;

    void WriteData();

};


#endif //BT_ROBOTICS_BENCHMARKER_H
