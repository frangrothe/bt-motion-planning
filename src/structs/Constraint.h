//
// Created by francesco on 19.05.21.
//

#ifndef BT_ROBOTICS_CONSTRAINT_H
#define BT_ROBOTICS_CONSTRAINT_H

struct Constraint {
    double x_lb;
    double x_ub;
    double t_lb;
    double t_ub;

    Constraint(double xLb, double xUb, double tLb, double tUb) : x_lb(xLb), x_ub(xUb), t_lb(tLb), t_ub(tUb) {}
};


#endif //BT_ROBOTICS_CONSTRAINT_H
