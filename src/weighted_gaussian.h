#ifndef WEIGHTED_GAUSSIAN_H
#define WEIGHTED_GAUSSIAN_H

#include <thrust/functional.h>

#include "covariance_matrix.h"
#include "point.h"

namespace pclem {
    class WeightedGaussian {
    public:
        WeightedGaussian();
        WeightedGaussian(Point& mu, CovarianceMatrix& sigma);

        struct likelihood : public thrust::unary_function<Point,double> {
            __host__ __device__
            double operator()(Point p) {
                return p.x;
            }
        };

    private:
        Point mu;
        CovarianceMatrix sigma;
    };
}

#endif
