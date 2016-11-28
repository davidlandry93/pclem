
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
    DeviceHierarchicalGaussianMixture::DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture)
        : pcl(pcl), mixture(mixture), children() {}

    void DeviceHierarchicalGaussianMixture::expand_n_levels(int n_levels) {
        VLOG(10) << "Expanding hierarchy of n levels...";

        run_em();

        int n_nodes_to_create = std::pow(AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE, n_levels);
        std::deque<std::shared_ptr<DeviceHierarchicalGaussianMixture>> to_expand;

        this->create_children(to_expand);

        while(n_nodes_to_create > 0) {
            std::shared_ptr<DeviceHierarchicalGaussianMixture> current_node = to_expand.front();
            to_expand.pop_front();

            current_node->create_children(to_expand);

            n_nodes_to_create -= AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE;
            VLOG(11) << "Nodes to create: " << n_nodes_to_create;
            VLOG(11) << "Nodes in queue: " << to_expand.size();
        }

        VLOG(10) << "Done expanding hierarchy of n levels.";
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

    void DeviceHierarchicalGaussianMixture::create_children(std::deque<std::shared_ptr<DeviceHierarchicalGaussianMixture>>& to_expand) {
        VLOG(10) << "Creating children of hierarchical gaussian mixture...";

        SortByBestAssociationOperation op;
        auto boundaries = pcl.execute_pointcloud_operation(op);

        for(int i=0; i < boundaries.size() - 1; i++) {
            DevicePointCloud child_pcl;

            std::cout << "Boundaries found: " << boundaries[i] << " ";
            std::cout << boundaries[i+1] << std::endl;

            child_pcl.set_points(pcl.get_data(), pcl.begin() + boundaries[i], pcl.begin() + boundaries[i+1]);

            WeightedGaussian current_gaussian = mixture.get_gaussian(i);

            if(current_gaussian.get_weight() > MIN_WEIGHT_TO_PROCREATE) {
                GaussianMixtureFactory gmm_factory;
                GaussianMixture child_mixture = gmm_factory.around_point(current_gaussian.get_mu(),
                                                                         current_gaussian.get_sigma(),
                                                                         AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE,
                                                                         UNIFORM_DISTRIBUTION_SIZE);

                auto child = std::shared_ptr<DeviceHierarchicalGaussianMixture>(new DeviceHierarchicalGaussianMixture(child_pcl, child_mixture));
                child->run_em();

                children.push_back(child);
                to_expand.push_back(child);
            }
        }

        VLOG(10) << "Done creating children";
    }

    DeviceHierarchicalGaussianMixture DeviceHierarchicalGaussianMixture::create_one_child(const DevicePointCloud& child_pcl,
                                                                                          const WeightedGaussian& parent_distribution) const {
        VLOG(10) << "Creating child of hierarchical gaussian mixture...";

        GaussianMixtureFactory gmm_factory;

        GaussianMixture child_mixture = gmm_factory.around_point(parent_distribution.get_mu(),
                                                                 parent_distribution.get_sigma(),
                                                                 AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE,
                                                                 UNIFORM_DISTRIBUTION_SIZE);

        DeviceHierarchicalGaussianMixture child(child_pcl, child_mixture);
        child.run_em();

        VLOG(10) << "Don creating child of hierarchical gaussian mixture.";
        return child;
    }

    std::ostream& operator<<(std::ostream& os, const DeviceHierarchicalGaussianMixture& hierarchy) {
        hierarchy.print_with_padding(os, 0);
        return os;
    }

    void DeviceHierarchicalGaussianMixture::print_with_padding(std::ostream& os, int padding) const {
        for(int i = 0; i < padding; i++) {
            os << " ";
        }
        os << mixture.n_gaussians() << std::endl;

        for(std::shared_ptr<DeviceHierarchicalGaussianMixture> child: children) {
            child->print_with_padding(os, padding+1);
        }
    }
}
