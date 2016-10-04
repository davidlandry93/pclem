#ifndef PCLEM_POINT_H
#define PCLEM_POINT_H

#include <iostream>

namespace pclem {
    class Point {
    public:
        double x,y,z;

        __host__ __device__
        Point() : x(0), y(0), z(0) {}

        __host__ __device__
        Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

        __host__ __device__
        Point(const Point& other) : x(other.x), y(other.y), z(other.z) {}

        __host__ __device__
        Point operator-(const Point& other) {
            return Point(x - other.x, y - other.y, z - other.z);
        }

        double get_x() const { return x; }
        double get_y() const { return y; }
        double get_z() const { return z; }

        friend std::ostream& operator<<(std::ostream& os, const Point& p) {
            os << "(" << p.get_x() << ","
               << p.get_y() << ","
               << p.get_z() << ")";
            return os;
        }
    };
}

#endif
