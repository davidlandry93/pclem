#ifndef WEIGHTED_GAUSSIANS_H
#define WEIGHTED_GAUSSIANS_H

#include <vector>
#include <iostream>

#include "covariance_matrix.cuh"
#include "weighted_gaussian.cuh"
#include "raw_covariance_matrix.h"

namespace pclem {
    class PrivateGaussianMixture {
    public:
        PrivateGaussianMixture();
        PrivateGaussianMixture(std::vector<WeightedGaussian> gaussians);
        PrivateGaussianMixture(PrivateGaussianMixture&& other);
        PrivateGaussianMixture& operator=(PrivateGaussianMixture&& other);
        std::vector<WeightedGaussian>::const_iterator begin() const;
        std::vector<WeightedGaussian>::const_iterator end() const;
        int n_gaussians() const;
        WeightedGaussian get_gaussian(int i) const; 
        friend std::ostream& operator<<(std::ostream& os, const PrivateGaussianMixture& mixture);

    private:
        std::vector<WeightedGaussian> gaussians;
    };
}

#endif
