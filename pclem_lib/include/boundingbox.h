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
        Point getMin() const;
        Point getMax() const;
        void setMin(Point min);
        void setMax(Point max);
        std::vector<Point> corners() const;
        BoundingBox& operator=(const BoundingBox& other);
        double volume() const;

    private:
        Point min;
        Point max;
    };
}

#endif
