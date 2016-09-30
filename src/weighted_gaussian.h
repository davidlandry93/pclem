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
        CovarianceMatrix get_sigma();


    private:
        Point mu;
        CovarianceMatrix sigma;
    };
}

#endif
