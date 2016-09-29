#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <vector>

#include "point.cuh"

namespace pclem {
    class BoundingBox {
    public:
        BoundingBox();
        BoundingBox(Point min, Point max);
        void setMin(Point min);
        void setMax(Point max);
        std::vector<Point> corners();

    private:
        Point min;
        Point max;
    };
}

#endif
