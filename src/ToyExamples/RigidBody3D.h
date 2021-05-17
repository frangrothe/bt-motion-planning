//
// Created by francesco on 12.05.21.
//
#include <iostream>
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/geometric/SimpleSetup.h"

#ifndef BT_ROBOTICS_RIGIDBODY3D_H
#define BT_ROBOTICS_RIGIDBODY3D_H

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rb3d {

bool isStateValid(const ob::State *state);
void planRigidBody3D();

}



#endif //BT_ROBOTICS_RIGIDBODY3D_H
