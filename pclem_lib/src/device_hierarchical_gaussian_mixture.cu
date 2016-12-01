
#include <memory>
#include <glog/logging.h>
#include <thrust/partition.h>
#include <thrust/find.h>
#include <deque>

#include "em_algorithm.h"
#include "associated_point.cuh"
#include "device_hierarchical_gaussian_mixture.h"
#include "pointcloud.h"
#include "gaussian_mixture_factory.h"
#include "sort_by_best_association_operation.h"

namespace pclem {
    DeviceHierarchicalGaussianMixture::DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture, const WeightedGaussian& parent, const std::shared_ptr<std::vector<DeviceHierarchicalGaussianMixture>>& node_vector)
        : pcl(pcl),
          mixture(mixture),
          children(),
          parent_distribution(parent),
          node_vector(node_vector),
          level_boundaries(),
          children_begin(node_vector->end()),
          children_end(node_vector->end()){}

    void DeviceHierarchicalGaussianMixture::expand_n_levels(int n_levels) {
        run_em();

        auto new_level_begin = node_vector->end();

        create_children();

        for(auto it = new_level_begin; it != node_vector->end(); it++) {
            it->expand_n_levels(n_levels-1);
        }
    }

    void DeviceHierarchicalGaussianMixture::run_em() {
        VLOG(10) << "Running em of DeviceHierarchicalGaussianMixture...";

        auto ptr = std::shared_ptr<DevicePointCloud>(new DevicePointCloud(pcl));
        PointCloud vanilla_pcl(ptr);

        EmAlgorithm em(vanilla_pcl, mixture);
        em.run(EM_CONVERGENCE_THRESHOLD);

        mixture = em.get_mixture();

        VLOG(10) << "Done running em of DeviceHierarchicalGaussianMixture";
    }

    void DeviceHierarchicalGaussianMixture::create_children() {
        children_begin = node_vector->end();

        if(mixture.n_nonzero_gaussians() < MIN_DISTRIBUTIONS_TO_PROCREATE) {
            children_end = node_vector->end();
            VLOG(2) << "Not enough gaussians in mixture to create children.";
        }

        // Sort the points by best association.
        SortByBestAssociationOperation op;
        auto partition_of_points = pcl.execute_pointcloud_operation(op);

        // For every pair of boundaries, create a point cloud and the according HGMM.
        for(int i=0; i < partition_of_points.size() - 1; i++) {
            WeightedGaussian current_gaussian = mixture.get_gaussian(i);

            if(current_gaussian.get_weight() < MIN_WEIGHT_TO_PROCREATE) {
                VLOG(2) << "Parent gaussian has too little weight to create children.";
            } else {
                DevicePointCloud child_pcl;
                child_pcl.set_points(pcl.get_data(), pcl.begin() + partition_of_points[i], pcl.begin() + partition_of_points[i+1]);

                GaussianMixtureFactory gmm_factory;
                GaussianMixture child_mixture = gmm_factory.around_point(current_gaussian.get_mu(),
                                                                         current_gaussian.get_sigma(),
                                                                         AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE,
                                                                         UNIFORM_DISTRIBUTION_SIZE);

                node_vector->push_back(DeviceHierarchicalGaussianMixture(child_pcl, child_mixture, current_gaussian, node_vector));
            }
        }

        children_end = node_vector->end();
        VLOG(3) << "Created " << children_end - children_begin << " children.";
    }

    std::ostream& operator<<(std::ostream& os, const DeviceHierarchicalGaussianMixture& hierarchy) {
        hierarchy.print_with_padding(os, 0);
        return os;
    }

    void DeviceHierarchicalGaussianMixture::print_with_padding(std::ostream& os, int padding) const {
        for(int i = 0; i < padding; i++) {
            os << " ";
        }
        os << mixture.n_nonzero_gaussians() << std::endl;

        for(std::shared_ptr<DeviceHierarchicalGaussianMixture> child: children) {
            child->print_with_padding(os, padding+1);
        }
    }

    void DeviceHierarchicalGaussianMixture::get_leaves(std::vector<WeightedGaussian>& leaves) const {
        std::cout << children.size() << std::endl;
        if(children.size() == 0) {
            leaves.push_back(parent_distribution);
        } else {
            for(std::shared_ptr<DeviceHierarchicalGaussianMixture> child : children) {
                child->get_leaves(leaves);
            }
        }
    }
}
