
#include "pclem_math.h"
#include "ellipsoid.h"

namespace pclem {
    Ellipsoid::Ellipsoid() :
        a(0), b(0), c(0), position(0,0,0), rotation(), opacity(1.0) {}

    Ellipsoid::Ellipsoid(double a, double b, double c, const Vector3& position, const Matrix33& rotation, const double& opacity) :
        a(a), b(b), c(c), position(position), rotation(rotation), opacity(opacity) {}

    std::ostream& operator<<(std::ostream& os, const Ellipsoid& e) {
        os << "Axes: [" << e.a << "," << e.b << "," << e.c <<
            "] Translation: " << e.position << "Rotation: " << e.rotation << "Opacity: " << e.opacity;
        return os;
    }

    std::ofstream& operator<<(std::ofstream& ofs, const Ellipsoid& e) {
        ofs << e.a << "," << e.b << "," << e.c << "," <<
            e.position << "," << e.rotation << "," << e.opacity << std::endl;

        return ofs;
    }

    bool Ellipsoid::operator==(const Ellipsoid& other) const {
        return approximatelyEqual(a, other.a, 1e-6) &&
            approximatelyEqual(b, other.b, 1e-6) &&
            approximatelyEqual(c, other.c, 1e-6) &&
            approximatelyEqual(opacity, other.opacity, 1e-6) &&
            position == other.position &&
            rotation == other.rotation;
    }
}
