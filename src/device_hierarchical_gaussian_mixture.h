#ifndef DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H
#define DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H

#include <thrust/device_vector.h>

#include "associated_point.cuh"
#include "gaussian_mixture.h"
#include "device_pointcloud.h"

namespace pclem {
    class DeviceHierarchicalGaussianMixture {
    public:
        DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture);
        void create_children();
    private:
        DevicePointCloud pcl;
        GaussianMixture mixture;
        std::vector<DeviceHierarchicalGaussianMixture> children;
    };
}

#endif
