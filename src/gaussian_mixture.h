#ifndef WEIGHTED_GAUSSIANS_H
#define WEIGHTED_GAUSSIANS_H

#include <thrust/device_vector.h>

#include "point.h"
#include "covariance_matrix.h"

namespace pclem {
    struct WeightedGaussian {
        Point mu;
        CovarianceMatrix sigma;
        double weight;

        WeightedGaussian(Point mu, CovarianceMatrix sigma) :
            mu(std::move(mu)), sigma(std::move(sigma)) {

        }
    };

    class GaussianMixture {
    public:
        GaussianMixture();
        GaussianMixture(std::vector<WeightedGaussian> gaussians);
    private:
        thrust::device_vector<WeightedGaussian> gaussians;
    };
}

#endif
