#ifndef EM_ALGORITHM_H
#define EM_ALGORITHM_H

#include <memory>

#include <thrust/device_vector.h>

#include "private_gaussian_mixture.h"

namespace pclem {
    class DevicePointCloud;
    class EmAlgorithm {
    public:
        EmAlgorithm(EmAlgorithm&& other);
        static EmAlgorithm from_pcl(DevicePointCloud& pcl);
        void expectation();
        void maximization();
        double log_likelihood();
        EmAlgorithm& operator=(EmAlgorithm&& other);
        friend std::ostream& operator<<(std::ostream& os, const EmAlgorithm& em);
    private:
        EmAlgorithm(DevicePointCloud& pcl, PrivateGaussianMixture& mixture);
        DevicePointCloud pcl;
        PrivateGaussianMixture mixture;
    };
}

#endif
