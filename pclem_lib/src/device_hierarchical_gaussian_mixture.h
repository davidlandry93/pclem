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
#include "visualization.h"

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
        void expand_n_levels(int n_levels, double em_convergence_threshold);
        void run_em(double em_convergence_threshold);
        void get_leaves(std::vector<WeightedGaussian>& leaves) const;
        double log_likelihood_of_pointcloud(const PointCloud& pointcloud) const;

        friend std::ostream& operator<<(std::ostream& os, const DeviceHierarchicalGaussianMixture& hierarchy);
        void insert_into_visualization(Visualization& vis) const;

    private:

        const double UNIFORM_DISTRIBUTION_SIZE = 2.5;
        const double DEFAULT_EM_CONVERGENCE_THRESHOLD = 0.01;
        const double MIN_WEIGHT_TO_PROCREATE = 1e-15;
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
