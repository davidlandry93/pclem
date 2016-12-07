#ifndef ASSOCIATED_POINT_H
#define ASSOCIATED_POINT_H

#include "point.h"
#include "device_point.cuh"

namespace pclem {

    struct AssociatedPoint : public DevicePoint {
        static const int N_DISTRIBUTIONS_PER_MIXTURE = 9;

        double associations[N_DISTRIBUTIONS_PER_MIXTURE];
        int best_distribution;

        __host__ __device__
        AssociatedPoint() : DevicePoint(), associations{0.0} {}

        __host__ __device__
        AssociatedPoint(double _x, double _y, double _z) :
            DevicePoint(_x,_y,_z), associations{0.0} {}

        __host__ __device__
        AssociatedPoint(const Point& other) : DevicePoint(other.x,other.y,other.z), best_distribution(0) {}

        Point to_host() const {
            return Point(x,y,z);
        }

        friend std::ostream& operator<<(std::ostream& os, const AssociatedPoint& p) {
            os << "(" << p.x << p.y << p.z << "). Associations: [";
            for(int i = 0; i < N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                os << p.associations[i] << ",";
            }
            os << "]";

            return os;
        }
    };
}

#endif
