#ifndef EM_ALGORITHM_H
#define EM_ALGORITHM_H

#include <memory>

#include <thrust/device_vector.h>

#include "pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class EmAlgorithm {
    public:
        EmAlgorithm(EmAlgorithm&& other);
        static EmAlgorithm from_pcl(PointCloud& pcl);
        void expectation();
        void maximization();
        EmAlgorithm& operator=(EmAlgorithm&& other);
        friend std::ostream& operator<<(std::ostream& os, const EmAlgorithm& em);
    private:
        EmAlgorithm(PointCloud& pcl, GaussianMixture& mixture, std::vector<double>& likelihoods);
        PointCloud pcl;
        GaussianMixture mixture;
        thrust::device_vector<double> likelihoods;
    };
}

#endif
