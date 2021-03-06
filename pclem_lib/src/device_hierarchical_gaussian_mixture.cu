
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
    DeviceHierarchicalGaussianMixture::DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture, const WeightedGaussian& parent, const NodeVector& node_vector)
        : pcl(pcl),
          mixture(mixture),
          children(),
          parent_distribution(parent),
          node_vector(node_vector),
          level_boundaries() {}

    void DeviceHierarchicalGaussianMixture::expand_n_levels(int n_levels) {
        expand_n_levels(n_levels, DEFAULT_EM_CONVERGENCE_THRESHOLD);
    }

    void DeviceHierarchicalGaussianMixture::expand_n_levels(int n_levels, double em_convergence_threshold) {
        VLOG(2) << "Expanding " << n_levels << " level...";

        if(pcl.get_n_points() == 0) {
            VLOG(9) << "No point in PCL. Done expanding n level.";
            return;
        }

        run_em(em_convergence_threshold);

        if(n_levels == 0) {
            return;
        }

        auto size_before_expanding = node_vector->size();
        create_children();
        auto size_after_expanding = node_vector->size();

        for(auto i = size_before_expanding; i < size_after_expanding; i++) {
            (*node_vector)[i]->expand_n_levels(n_levels-1, em_convergence_threshold);
        }

        VLOG(2) << "Done expanding n levels.";
    }

    void DeviceHierarchicalGaussianMixture::run_em(double em_convergence_threshold) {
        VLOG(10) << "Running em of DeviceHierarchicalGaussianMixture...";

        auto ptr = std::shared_ptr<DevicePointCloud>(new DevicePointCloud(pcl));
        PointCloud vanilla_pcl(ptr);

        EmAlgorithm em(vanilla_pcl, mixture);
        em.run(em_convergence_threshold);

        mixture = em.get_mixture();

        VLOG(10) << "Done running em of DeviceHierarchicalGaussianMixture";
    }

    void DeviceHierarchicalGaussianMixture::create_children() {
        VLOG(7) << "Creating children of mixture...";

        if(mixture.n_nonzero_gaussians() < MIN_DISTRIBUTIONS_TO_PROCREATE) {
            VLOG(2) << "Not enough gaussians in mixture to create children.";
        }

        // Sort the points by best association.
        SortByBestAssociationOperation op;
        auto partition_of_points = pcl.execute_pointcloud_operation(op);

        // For every pair of boundaries, create a point cloud and the according HGMM.
        for(int i=0; i < partition_of_points.size() - 1; i++) {
            VLOG(2) << "Creating distribution " << i;
            WeightedGaussian current_gaussian = mixture.get_gaussian(i);

            if(current_gaussian.get_weight() < MIN_WEIGHT_TO_PROCREATE) {
                VLOG(2) << "Parent gaussian has too little weight to create children.";
            } else if (partition_of_points[i] == partition_of_points[i+1]){
                VLOG(2) << "Parent gaussian had no point assigned to it.";
            } else {
                DevicePointCloud child_pcl(current_gaussian.weight_in_hierarchy());
                VLOG(2) << "Points from " << partition_of_points[i] << " to " << partition_of_points[i+1];
                child_pcl.set_points(pcl.get_data(), pcl.begin() + partition_of_points[i], pcl.begin() + partition_of_points[i+1]);

                GaussianMixtureFactory gmm_factory;
                GaussianMixture child_mixture = gmm_factory.from_pcl_corners(child_pcl, current_gaussian.weight_in_hierarchy());

                auto child = std::shared_ptr<DeviceHierarchicalGaussianMixture>(new DeviceHierarchicalGaussianMixture(child_pcl, child_mixture, current_gaussian, node_vector));
                node_vector->push_back(child);
                children.push_back(child);
            }
        }

        VLOG(7) << "Done creating children of mixture.";
    }

    std::ostream& operator<<(std::ostream& os, const DeviceHierarchicalGaussianMixture& hierarchy) {
        hierarchy.print_with_padding(os, 0);
        return os;
    }

    void DeviceHierarchicalGaussianMixture::print_with_padding(std::ostream& os, int padding) const {
        for(int i = 0; i < padding; i++) {
            os << " ";
        }
        os << "(" << parent_distribution.get_weight() << ")" << mixture.n_nonzero_gaussians() << std::endl;

        for(std::shared_ptr<DeviceHierarchicalGaussianMixture> child: children) {
            child->print_with_padding(os, padding+1);
        }
    }

    void DeviceHierarchicalGaussianMixture::get_leaves(std::vector<WeightedGaussian>& leaves) const {
        if(children.size() == 0) {
            leaves.push_back(parent_distribution);
        } else {
            for(std::shared_ptr<DeviceHierarchicalGaussianMixture> child : children) {
                child->get_leaves(leaves);
            }
        }
    }

    double DeviceHierarchicalGaussianMixture::log_likelihood_of_pointcloud(PointCloud& pointcloud) const {
        if(!children.empty()) {
            double log_likelihood = 0.0;

            for(auto const& child : children) {
                double weight_of_parent = parent_distribution.get_weight();
                double log_likelihood_of_child = child->log_likelihood_of_pointcloud(pointcloud);

                log_likelihood = weight_of_parent * log_likelihood_of_child;
            }

            return log_likelihood;
        } else {
            pointcloud.compute_associations(mixture);
            // std::cout << "Mixture: " << std::endl << mixture;
            double log_of_mixture = pointcloud.log_likelihood_of_mixture(mixture);

            std::cout << "Log of mixture. " << log_of_mixture << std::endl;

            return log_of_mixture;
        }
    }

    void DeviceHierarchicalGaussianMixture::insert_into_visualization(Visualization& vis) const {
        std::vector<WeightedGaussian> leaves;
        get_leaves(leaves);

        for(auto leaf : leaves) {
            leaf.insert_into_visualization(vis);
        }
    }
}
