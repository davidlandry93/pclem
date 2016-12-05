#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <iostream>
#include "vector3.h"
#include "matrix33.h"

namespace pclem {
    class Ellipsoid {
    public:
        double a,b,c; // The length of the axes.
        Vector3 position;
        Matrix33 rotation;
        double opacity;

        Ellipsoid() : a(0), b(0), c(0), position(0,0,0), rotation(), opacity(1.0) {}
        Ellipsoid(double a, double b, double c, const Vector3& position, const Matrix33& rotation, const double& opacity) :
            a(a), b(b), c(c), position(position), rotation(rotation), opacity(opacity) {}

        friend std::ostream& operator<<(std::ostream& os, const Ellipsoid& e) {
            os << "Axes: [" << e.a << "," << e.b << "," << e.c <<
                "] Translation: " << e.position << "Rotation: " << e.rotation << "Opacity: " << e.opacity;
            return os;
        }
    };
}

#endif
