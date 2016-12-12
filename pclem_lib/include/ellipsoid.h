#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <iostream>
#include <fstream>
#include "vector3.h"
#include "matrix33.h"

namespace pclem {
    class Ellipsoid {
    public:
        double a,b,c; // The length of the axes.
        Vector3 position;
        Matrix33 rotation;
        double opacity;

        Ellipsoid();
        Ellipsoid(double a, double b, double c, const Vector3& position, const Matrix33& rotation, const double& opacity);

        bool operator==(const Ellipsoid& other) const;
        friend std::ofstream& operator<<(std::ofstream& ofs, const Ellipsoid& e);
        friend std::ostream& operator<<(std::ostream& os, const Ellipsoid& e);
    };
}

#endif
