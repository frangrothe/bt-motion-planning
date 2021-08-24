#include <utility>

//
// Created by francesco on 09.08.21.
//

#ifndef BT_ROBOTICS_TIMENDCONSTRAINT_H
#define BT_ROBOTICS_TIMENDCONSTRAINT_H

namespace nd {

struct TimeNDConstraint {
    TimeNDConstraint(double radius, std::vector<std::vector<double>> path) : radius(radius), path(std::move(path)) {}

    double radius;
    std::vector<std::vector<double>> path;
};
}

#endif //BT_ROBOTICS_TIMENDCONSTRAINT_H
