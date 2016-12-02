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

        Ellipsoid() : a(0), b(0), c(0), position(0,0,0), rotation() {}
        Ellipsoid(double a, double b, double c, const Vector3& position, const Matrix33& rotation) :
            a(a), b(b), c(c), position(position), rotation(rotation) {}

        friend std::ostream& operator<<(std::ostream& os, const Ellipsoid& e) {
            os << "Axes: [" << e.a << "," << e.b << "," << e.c <<
                "] Translation: " << e.position << "Rotation: " << e.rotation;
            return os;
        }
    };
}

#endif
