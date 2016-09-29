#ifndef WEIGHTED_GAUSSIANS_H
#define WEIGHTED_GAUSSIANS_H

#include <thrust/device_vector.h>

#include "point.h"
#include "covariance_matrix.h"
#include "weighted_gaussian.h"

namespace pclem {
    class GaussianMixture {
    public:
        GaussianMixture();
        GaussianMixture(std::vector<WeightedGaussian> gaussians);
        const thrust::device_vector<WeightedGaussian>& get_gaussians() const;
        int n_gaussians() const;
    private:
        thrust::device_vector<WeightedGaussian> gaussians;
    };
}

#endif