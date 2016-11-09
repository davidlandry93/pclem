
#include "boundingbox.h"

namespace pclem {
    BoundingBox::BoundingBox() :
        min(Point(0.0,0.0,0.0)), max(Point(0.0,0.0,0.0)) {
    }

    BoundingBox::BoundingBox(Point min, Point max) :
        min(min), max(max) {
    }

    BoundingBox::BoundingBox(const BoundingBox& other) :
        min(other.min), max(other.max) {}

    void BoundingBox::setMin(Point pmin) {
        min = pmin;
    }

    void BoundingBox::setMax(Point pmax) {
        max = pmax;
    }

    std::vector<Point> BoundingBox::corners() const {
        auto corners = std::vector<Point>();

        corners.push_back(Point(max.x, max.y, max.z));
        corners.push_back(Point(max.x, max.y, min.z));
        corners.push_back(Point(max.x, min.y, max.z));
        corners.push_back(Point(min.x, max.y, max.z));
        corners.push_back(Point(max.x, min.y, min.z));
        corners.push_back(Point(min.x, min.y, max.z));
        corners.push_back(Point(min.x, max.y, min.z));
        corners.push_back(Point(min.x, min.y, min.z));

        return corners;
    }
}
