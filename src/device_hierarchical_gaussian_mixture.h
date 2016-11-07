#ifndef DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H
#define DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H

#include <thrust/device_vector.h>

#include "associated_point.cuh"
#include "pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class DeviceHierarchicalGaussianMixture{
    public:
        DeviceHierarchicalGaussianMixture(const thrust::device_vector<AssociatedPoint>::iterator& pcl_begin,
                                          const thrust::device_vector<AssociatedPoint>::iterator& pcl_end,
                                          GaussianMixture& mixture);
        void create_children();
    private:
        thrust::device_vector<AssociatedPoint>::iterator pcl_begin;
        thrust::device_vector<AssociatedPoint>::iterator pcl_end;
        GaussianMixture mixture;
        std::vector<DeviceHierarchicalGaussianMixture> children;
    };
}

#endif
