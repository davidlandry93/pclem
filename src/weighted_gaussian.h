#ifndef WEIGHTED_GAUSSIAN_H
#define WEIGHTED_GAUSSIAN_H

#include "covariance_matrix.h"
#include "point.h"

namespace pclem {
    class WeightedGaussian {
    public:
        WeightedGaussian();
        WeightedGaussian(Point& mu, CovarianceMatrix& sigma);

        WeightedGaussian(const WeightedGaussian& other);

        CovarianceMatrix get_sigma() const;

    private:
        Point mu;
        CovarianceMatrix sigma;
    };
}

#endif
