#ifndef EM_ALGORITHM_H
#define EM_ALGORITHM_H

#include <memory>

#include <thrust/device_vector.h>

#include "pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class EmAlgorithm {
    public:
        EmAlgorithm(PointCloud pcl);
        void expectation();
        void maximization();
    private:
        PointCloud pcl;
        GaussianMixture mixture;
        thrust::device_vector<double> likelihoods;
    };
}

#endif
