#ifndef POINT_H
#define POINT_H

#include <iostream>

namespace pclem {
    struct Point {
        double x,y,z;

        Point() : x(0), y(0), z(0) {}
        Point(double x, double y, double z) : x(x), y(y), z(z) {}
        Point(const Point& other) : x(other.x), y(other.y), z(other.z) {}

        friend std::ostream& operator<<(std::ostream& os, const Point& p) {
            os << "(" << p.x << ","
               << p.y << ","
               << p.z << ")";
            return os;
        }
    };
}

#endif
