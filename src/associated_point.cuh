#ifndef ASSOCIATED_POINT_H
#define ASSOCIATED_POINT_H

#include "point.h"
#include "device_point.cuh"

namespace pclem {

    struct AssociatedPoint : public DevicePoint {
        static const int N_DISTRIBUTIONS_PER_MIXTURE = 8;

        double likelihoods[N_DISTRIBUTIONS_PER_MIXTURE];

        __host__ __device__
        AssociatedPoint() : DevicePoint(), likelihoods{0.0} {}

        AssociatedPoint(double _x, double _y, double _z) :
            DevicePoint(_x,_y,_z), likelihoods{0.0} {}

        AssociatedPoint(const Point& other) : DevicePoint(other.x,other.y,other.z) {}

        Point to_host() const {
            return Point(x,y,z);
        }

        friend std::ostream& operator<<(std::ostream& os, const AssociatedPoint& p) {
            os << "(" << p.x << p.y << p.z << "). Associations: [";
            for(int i = 0; i < N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                os << p.likelihoods[i] << ",";
            }
            os << "]";

            return os;
        }
    };
}

#endif
