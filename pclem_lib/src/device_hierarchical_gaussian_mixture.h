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
        typedef std::shared_ptr<std::vector<std::shared_ptr<DeviceHierarchicalGaussianMixture>>> NodeVector;

        DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl,
                                          const GaussianMixture& mixture,
                                          const WeightedGaussian& parent_mixture,
                                          const NodeVector& node_vector);
        void create_children();
        void expand_n_levels(int n_levels);
        void run_em();

        friend std::ostream& operator<<(std::ostream& os, const DeviceHierarchicalGaussianMixture& hierarchy);
        void get_leaves(std::vector<WeightedGaussian>& leaves) const;

    private:

        const double UNIFORM_DISTRIBUTION_SIZE = 2.5;
        const double EM_CONVERGENCE_THRESHOLD = 0.1;
        const double MIN_WEIGHT_TO_PROCREATE = 1e-10;
        const double MIN_DISTRIBUTIONS_TO_PROCREATE = 2;

        DevicePointCloud pcl;
        GaussianMixture mixture;
        std::vector<std::shared_ptr<DeviceHierarchicalGaussianMixture>> children;
        WeightedGaussian parent_distribution;
        NodeVector node_vector;
        std::vector<int> level_boundaries;

        void print_with_padding(std::ostream& os, int padding) const;
        void expand_one_level();
    };
}

#endif
