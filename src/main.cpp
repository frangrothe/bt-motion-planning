#include <iostream>

#include "Time1D/Time1DPlanner.h"
#include "Time2D/Time2DPlanner.h"
#include "Dubins/DubinsPlanner.h"
#include "auxillary.h"

int main(int argc, char* argv[]) {

    std::string filename;
    int plannerType = 0;
    if (argc < 2) {
        filename = auxillary::currentDateTime();
    }
    else {
        filename = argv[1];
    }
    if (argc > 2) {
        plannerType = std::stoi(argv[2]);
    }

    std::cout << "Planner Type is " << plannerType;
    std::cout << "\nFilename is: " << filename << std::endl;

    switch (plannerType) {
        case 1: {
            time_1d::Time1DPlanner planner{filename};
            planner.planMotion();
            break;
        }
        case 2: {
            time_2d::Time2DPlanner planner{filename};
            planner.planMotion();
            break;
        }
        case 3: {
            dubins::DubinsPlanner planner{filename};
            planner.planMotion();
            break;
        }
        default: {
            time_2d::Time2DPlanner planner{filename};
//            planner.planMotion();
            planner.test();
        }
    }

    return 0;
}
