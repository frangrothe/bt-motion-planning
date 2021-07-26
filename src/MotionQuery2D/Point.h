//
// Created by francesco on 19.07.21.
//

#ifndef BT_ROBOTICS_POINT_H
#define BT_ROBOTICS_POINT_H

namespace query2d {

struct Point {

public:
    Point(double x, double y, double t) : x(x), y(y), t(t) {}

    double x;
    double y;
    double t;
};
}

#endif //BT_ROBOTICS_POINT_H
