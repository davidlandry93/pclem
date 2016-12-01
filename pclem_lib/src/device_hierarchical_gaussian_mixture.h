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
        DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl,
                                          const GaussianMixture& mixture,
                                          const WeightedGaussian& parent_mixture,
                                          const std::shared_ptr<std::vector<DeviceHierarchicalGaussianMixture>>& node_vector);
        void create_children();
        void expand_n_levels(int n_levels);
        void run_em();

        friend std::ostream& operator<<(std::ostream& os, const DeviceHierarchicalGaussianMixture& hierarchy);
        void get_leaves(std::vector<WeightedGaussian>& leaves) const;

    private:
        const double UNIFORM_DISTRIBUTION_SIZE = 2.5;
        const double EM_CONVERGENCE_THRESHOLD = 0.001;
        const double MIN_WEIGHT_TO_PROCREATE = 1e-10;
        const double MIN_DISTRIBUTIONS_TO_PROCREATE = 2;

        DevicePointCloud pcl;
        GaussianMixture mixture;
        std::vector<std::shared_ptr<DeviceHierarchicalGaussianMixture>> children;
        WeightedGaussian parent_distribution;
        std::shared_ptr<std::vector<DeviceHierarchicalGaussianMixture>> node_vector;
        std::vector<std::vector<DeviceHierarchicalGaussianMixture>::iterator> level_boundaries;
        std::vector<DeviceHierarchicalGaussianMixture>::iterator children_begin;
        std::vector<DeviceHierarchicalGaussianMixture>::iterator children_end;

        void print_with_padding(std::ostream& os, int padding) const;
        void expand_one_level();
    };
}

#endif
