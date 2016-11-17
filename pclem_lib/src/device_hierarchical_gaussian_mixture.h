#ifndef DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H
#define DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H

#include <vector>
#include <deque>
#include <thrust/device_vector.h>

#include "gaussian_mixture.h"
#include "device_pointcloud.h"

namespace pclem {
    class DeviceHierarchicalGaussianMixture {
    public:
        DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture);
        void create_children(std::deque<DeviceHierarchicalGaussianMixture*>& to_expand);
        void expand_n_levels(int n_levels);
        void run_em();

    private:
        const double UNIFORM_DISTRIBUTION_SIZE = 0.5;
        const double EM_CONVERGENCE_THRESHOLD = 0.001;
        const double MIN_WEIGHT_TO_PROCREATE = 0.01;

        DevicePointCloud pcl;
        GaussianMixture mixture;
        std::vector<DeviceHierarchicalGaussianMixture> children;

        DeviceHierarchicalGaussianMixture create_one_child(const DevicePointCloud& device_pcl,
                                                           const WeightedGaussian& parent_distribution) const;
    };
}

#endif
