#ifndef LIKELIHOOD_MATRIX_H
#define LIKELIHOOD_MATRIX_H

#include <thrust/device_vector.h>

#include "pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class LikelihoodMatrix {
    public:
        static LikelihoodMatrix build(const PointCloud& pcl, const GaussianMixture& mixture);
    private:
        thrust::device_vector<double> likelihoods;

        LikelihoodMatrix(thrust::device_vector<double> likelihoods);
    };
}

#endif
