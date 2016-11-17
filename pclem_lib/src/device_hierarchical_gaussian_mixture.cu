
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
        std::deque<DeviceHierarchicalGaussianMixture*> to_expand;

        to_expand.push_back(this);
        while(n_nodes_to_create > 0) {
            DeviceHierarchicalGaussianMixture* current_node = to_expand.front();
            to_expand.pop_front();

            current_node->create_children(to_expand);

            n_nodes_to_create -= AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE;
            VLOG(11) << "Nodes to create: " << n_nodes_to_create;
            VLOG(11) << "Nodes in queue: " << to_expand.size();
        }

        VLOG(10) << "Done expanding hierarchy of n levels.";
    }

    void DeviceHierarchicalGaussianMixture::run_em() {
        auto ptr = std::shared_ptr<DevicePointCloud>(new DevicePointCloud(pcl));
        PointCloud vanilla_pcl(ptr);

        auto em = EmAlgorithm::from_pcl(vanilla_pcl);
        em.run(EM_CONVERGENCE_THRESHOLD);
    }

    void DeviceHierarchicalGaussianMixture::create_children(std::deque<DeviceHierarchicalGaussianMixture*>& to_expand) {
        VLOG(10) << "Creating children of hierarchical gaussian mixture...";

        SortByBestAssociationOperation op;
        std::vector<DevicePointCloud::PointIterator> boundaries = pcl.execute_pointcloud_operation(op);

        for(int i=0; i < boundaries.size() - 1; i++) {
            DevicePointCloud child_pcl;
            child_pcl.set_points(pcl.get_data(), boundaries[i], boundaries[i+1]);

            children.push_back(create_one_child(child_pcl, mixture.get_gaussian(i)));
            to_expand.push_back(&children.back());
        }

        VLOG(10) << "Done creating children";
    }

    DeviceHierarchicalGaussianMixture DeviceHierarchicalGaussianMixture::create_one_child(const DevicePointCloud& child_pcl,
                                                                                          const WeightedGaussian& parent_distribution) const {
        GaussianMixtureFactory gmm_factory;

        GaussianMixture child_mixture = gmm_factory.around_point(parent_distribution.get_mu(),
                                                                 parent_distribution.get_sigma(),
                                                                 AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE,
                                                                 UNIFORM_DISTRIBUTION_SIZE);

        std::cout << "ChildMixture: " << child_mixture;

        DeviceHierarchicalGaussianMixture child(child_pcl, child_mixture);
        child.run_em();

        return child;
    }
}
