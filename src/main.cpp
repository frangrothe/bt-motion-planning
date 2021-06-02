#include <iostream>
#include "RigidBody3D.h"
#include "Time1DPlanner.h"
#include "SpaceTimePlanning/SpaceTimePlanner.h"

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

//    rb3d::planRigidBody3D();
//    t1d::Time1DPlanner planner{filename};
    SpaceTime::SpaceTimePlanner planner{filename};
    planner.planMotion();
//    planner.test();

    return 0;
}
