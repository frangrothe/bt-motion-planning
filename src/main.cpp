#include <iostream>
#include "RigidBody3D.h"
#include "Time1D.h"
#include "auxillary.h"

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
    tb1d::plan(filename);

    return 0;
}
