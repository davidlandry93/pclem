#ifndef PCLEM_POINT_H
#define PCLEM_POINT_H

namespace pclem {
    class Point {
    public:
        double x,y,z;

        __host__ __device__
        Point() : x(0), y(0), z(0) {}

        __host__ __device__
        Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    };
}

#endif
