#ifndef ASSOCIATED_POINT_H
#define ASSOCIATED_POINT_H

#include "point.cuh"

namespace pclem {
    class AssociatedPoint : public Point {
    public:
        static const int N_DISTRIBUTIONS_PER_MIXTURE = 8;

        double likelihoods[N_DISTRIBUTIONS_PER_MIXTURE];

        __host__ __device__
        AssociatedPoint() : Point(), likelihoods{0.0} {}

        AssociatedPoint(double _x, double _y, double _z) :
            Point(_x,_y,_z), likelihoods{0.0} {}

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
