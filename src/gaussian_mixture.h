#ifndef WEIGHTED_GAUSSIANS_H
#define WEIGHTED_GAUSSIANS_H

#include <iostream>
#include <thrust/device_vector.h>

#include "point.cuh"
#include "covariance_matrix.cuh"
#include "weighted_gaussian.cuh"
#include "pointcloud.h"
#include "raw_covariance_matrix.h"

namespace pclem {
    class GaussianMixture {
    public:
        GaussianMixture();
        GaussianMixture(std::vector<WeightedGaussian> gaussians);
        GaussianMixture(GaussianMixture&& other);
        GaussianMixture& operator=(GaussianMixture&& other);
        thrust::device_vector<WeightedGaussian>::const_iterator begin() const;
        thrust::device_vector<WeightedGaussian>::const_iterator end() const;
        int n_gaussians() const;
        WeightedGaussian get_gaussian(int i) const; 
        friend std::ostream& operator<<(std::ostream& os, const GaussianMixture& mixture);


    private:
        thrust::device_vector<WeightedGaussian> gaussians;

    };
}

#endif
