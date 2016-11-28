#ifndef DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H
#define DEVICE_HIERARCHICAL_GAUSSIAN_MIXTURE_NODE_H

#include <deque>
#include <vector>
#include <thrust/device_vector.h>
#include <iostream>
#include <memory>

#include "associated_point.cuh"
#include "gaussian_mixture.h"
#include "device_pointcloud.h"

namespace pclem {
    class DeviceHierarchicalGaussianMixture {
    public:
        DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture);
        void create_children(std::deque<std::shared_ptr<DeviceHierarchicalGaussianMixture>>& to_expand);
        void expand_n_levels(int n_levels);
        void run_em();

        friend std::ostream& operator<<(std::ostream& os, const DeviceHierarchicalGaussianMixture& hierarchy);

    private:
        const double UNIFORM_DISTRIBUTION_SIZE = 2.5;
        const double EM_CONVERGENCE_THRESHOLD = 0.001;
        const double MIN_WEIGHT_TO_PROCREATE = 1e-10;

        DevicePointCloud pcl;
        GaussianMixture mixture;
        std::vector<std::shared_ptr<DeviceHierarchicalGaussianMixture>> children;

        DeviceHierarchicalGaussianMixture create_one_child(const DevicePointCloud& device_pcl,
                                                           const WeightedGaussian& parent_distribution) const;

        void print_with_padding(std::ostream& os, int padding) const;
    };
}

#endif
