#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "point.h"
#include "ellipsoid.h"

namespace pclem {
    class Visualization {
    public:
        virtual void insert_point(const Point& point)=0;
        virtual void insert_ellipsoid(const Ellipsoid& ellipsoid)=0;
    };
}

#endif
