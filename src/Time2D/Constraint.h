//
// Created by francesco on 27.05.21.
//

#include <tuple>

#ifndef BT_ROBOTICS_CONSTRAINT2D_H
#define BT_ROBOTICS_CONSTRAINT2D_H

namespace time_2d {

struct Constraint {
    typedef std::function<std::tuple<double, double> (double, double, double)> MoveFunc;

public:
    Constraint(MoveFunc mf, double startX, double startY, double widthX = 0.1, double widthY = 0.1) :
            startX_(startX), startY_(startY), width_(widthX), height_(widthY), moveFunc_(mf) {};

    std::tuple<double, double, double, double> getBoundsForTime (double t) const {
        double x, y;
        std::tie(x, y) = moveFunc_(startX_, startY_, t);
        return std::make_tuple(x - width_ / 2, x + width_ / 2, y - height_ / 2, y + height_ / 2);
    }

    // returns true, if the two boxes collide
    bool checkAABBCollision(double aMinX, double aMaxX, double aMinY, double aMaxY, double t) const {
        double bMinX, bMaxX, bMinY, bMaxY;
        std::tie(bMinX, bMaxX, bMinY, bMaxY) = getBoundsForTime(t);
        return (aMinX <= bMaxX && aMaxX >= bMinX ) &&
                (aMinY <= bMaxY && aMaxY >= bMinY);
    }

private:

    double startX_;
    double startY_;

    double width_; // total width or x-axis length
    double height_; // total height or y-axis length
    MoveFunc moveFunc_;

};
}



#endif //BT_ROBOTICS_CONSTRAINT2D_H
