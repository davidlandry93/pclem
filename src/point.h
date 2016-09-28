#ifndef PCLEM_POINT_H
#define PCLEM_POINT_H

namespace pclem {
    struct Point {
        double x;
        double y;
        double z;

        Point(double x, double y, double z) :
            x(x), y(y), z(z) {
        }
    };
}

#endif
