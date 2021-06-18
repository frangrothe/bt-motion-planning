#include <iostream>
#include "Time1D/Time1DPlanner.h"
#include "Time2D/SpaceTimePlanner.h"

#include "auxillary.h"
#include "structs/Constraint2D.h"

int main(int argc, char* argv[]) {

    std::string filename;
    if (argc < 2) {
        filename = auxillary::currentDateTime();
    }
    else {
        filename = argv[1];
    }

    std::cout << "Filename is: " << filename << std::endl;

    time_1d::Time1DPlanner planner{filename};
//    time_2d::SpaceTimePlanner planner{filename};
    planner.planMotion();
//    planner.test();

    return 0;
}
