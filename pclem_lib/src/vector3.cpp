
#include <stdexcept>
#include <cmath>

#include "pclem_math.h"
#include "vector3.h"

namespace pclem {
    Vector3::Vector3() : x(0), y(0), z(0) {}

    Vector3::Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3::Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

    double Vector3::length() {
        return std::sqrt(x*x + y*y + z*z);
    }

    void Vector3::normalize() {
        double current_length = length();
        x /= current_length;
        y /= current_length;
        z /= current_length;
    }

    double& Vector3::operator[](int index) {
        switch(index) {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            throw std::range_error("Invalid index for vector3");
        }
    }

    void Vector3::operator=(const Vector3& other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    double Vector3::dot(const Vector3& other) const {
        return x*other.x + y*other.y + z*other.z;
    }

    std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << v.x << "," << v.y << "," << v.z;
        return os;
    }

    std::ofstream& operator<<(std::ofstream& ofs, const Vector3& v) {
        ofs << v.x << "," << v.y << "," << v.z;
        return ofs;
    }

    bool Vector3::operator==(const Vector3& other) const {
        return approximatelyEqual(x, other.x, 1e-10) &&
            approximatelyEqual(y, other.y, 1e-10) &&
            approximatelyEqual(z, other.z, 1e-10);
    }
}
