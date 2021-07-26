//
// Created by francesco on 19.07.21.
//

#include "QueryExecutor.h"

namespace query2d {

void QueryExecutor::execute() {

    std::string filename = "query";
    std::vector<std::vector<Point>> pathConstraints;

    std::vector<std::pair<double, double>> starts {
            {10.0, 5.0},
            {10.0, 45.0},
            {20.0, 5.0},
            {20.0, 45.0},
            {30.0, 5.0},
            {30.0, 45.0},
            {40.0, 5.0},
            {40.0, 45.0}
    };

    std::vector<std::pair<double, double>> goals {
            {22.0, 38.0},
            {22.0, 12.0},
            {28.0, 38.0},
            {28.0, 12.0},
            {34.0, 38.0},
            {34.0, 12.0},
            {16.0, 38.0},
            {16.0, 12.0}
    };

    for (int i = 0; i < starts.size(); ++i) {
        MotionQuery2DPlanner planner {starts[i].first, starts[i].second, goals[i].first, goals[i].second};
        if (i == 0)
            planner.writeConstraintsToJSON(filename);
        else
            planner.setPathConstraints(pathConstraints);
        planner.planMotion();
        planner.writeSolutionToJSON(filename, i);
        pathConstraints.push_back(planner.getPathConstraint());
    }

}
}