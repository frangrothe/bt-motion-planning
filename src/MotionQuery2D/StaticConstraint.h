//
// Created by francesco on 19.07.21.
//

#ifndef BT_ROBOTICS_STATICCONSTRAINT_H
#define BT_ROBOTICS_STATICCONSTRAINT_H

#endif //BT_ROBOTICS_STATICCONSTRAINT_H

namespace query2d {

struct StaticConstraint {

public:
    StaticConstraint(double xCentre, double yCentre, double radius) : x(xCentre), y(yCentre),
                                                                      radius(radius) {}
    double x;
    double y;
    double radius;

};
}