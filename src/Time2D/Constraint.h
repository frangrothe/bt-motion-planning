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
            startX_(startX), startY_(startY), widthX_(widthX), widthY_(widthY), moveFunc_(mf) {};

    std::tuple<double, double, double, double> getBoundsForTime (double t) const{
        double x, y;
        std::tie(x, y) = moveFunc_(startX_, startY_, t);
        return checkBounds(x - widthX_, x + widthX_, y - widthY_, y + widthY_);
    }

    void setBounds(double xLow, double xHigh, double yLow, double yHigh) {
        xBoundLow_ = xLow;
        xBoundHigh_ = xHigh;
        yBoundLow_ = yLow;
        yBoundHigh_ = yHigh;
    }

private:
    double xBoundLow_ = -std::numeric_limits<double>::infinity();
    double xBoundHigh_ = std::numeric_limits<double>::infinity();
    double yBoundLow_ = -std::numeric_limits<double>::infinity();
    double yBoundHigh_ = std::numeric_limits<double>::infinity();

    double startX_;
    double startY_;

    double widthX_;
    double widthY_;
    MoveFunc moveFunc_;

    std::tuple<double, double, double, double> checkBounds (double xLow, double xHigh, double yLow, double yHigh) const{
        xLow = xLow < xBoundLow_ ? xBoundLow_ : xLow;
        xLow = xLow > xBoundHigh_ ? xBoundHigh_ : xLow;
        xHigh = xHigh < xBoundLow_ ? xBoundLow_ : xHigh;
        xHigh = xHigh > xBoundHigh_ ? xBoundHigh_ : xHigh;

        yLow = yLow < yBoundLow_ ? yBoundLow_ : yLow;
        yLow = yLow > yBoundHigh_ ? yBoundHigh_ : yLow;
        yHigh = yHigh < yBoundLow_ ? yBoundLow_ : yHigh;
        yHigh = yHigh > yBoundHigh_ ? yBoundHigh_ : yHigh;

        return std::make_tuple(xLow, xHigh, yLow, yHigh);
    }

};
}



#endif //BT_ROBOTICS_CONSTRAINT2D_H
