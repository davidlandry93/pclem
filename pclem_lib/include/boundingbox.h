#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <vector>

#include "point.h"

namespace pclem {
    class BoundingBox {
    public:
        BoundingBox();
        BoundingBox(Point min, Point max);
        BoundingBox(const BoundingBox& other);
        void setMin(Point min);
        void setMax(Point max);
        std::vector<Point> corners() const;
        BoundingBox& operator=(const BoundingBox& other);

    private:
        Point min;
        Point max;
    };
}

#endif
