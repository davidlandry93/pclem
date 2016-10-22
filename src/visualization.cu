
#include <iostream>

#include "visualization.h"

namespace pclem {
    Visualization::Visualization() : points() {}

    void Visualization::insert_point(const Point& point) {
        points->InsertNextPoint(point.x, point.y, point.z);
    }

    void Visualization::insert_ellipsoid(const Ellipsoid& ellipsoid) {
        
    }

    void Visualization::visualize() {
        
    }
}
