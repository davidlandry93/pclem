
#include <memory>
#include <glog/logging.h>
#include <thrust/partition.h>
#include <thrust/find.h>

#include "em_algorithm.h"
#include "associated_point.cuh"
#include "device_hierarchical_gaussian_mixture.h"
#include "pointcloud.h"
#include "gaussian_mixture_factory.h"
#include "sort_by_best_association_operation.h"

namespace pclem {
    DeviceHierarchicalGaussianMixture::DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture)
        : pcl(pcl), mixture(mixture), children() {}

    void DeviceHierarchicalGaussianMixture::create_children(std::vector<DeviceHierarchicalGaussianMixture*>& to_expand) {
        VLOG(10) << "Creating children of hierarchical gaussian mixture...";
        SortByBestAssociationOperation op;

        std::vector<DevicePointCloud::PointIterator> boundaries = pcl.execute_pointcloud_operation(op);

        //children.push_back(create_one_child(sub_pcl_begin, first_unsorted, mixture.get_gaussian(i)));

        VLOG(10) << "Done creating children";
    }

    void DeviceHierarchicalGaussianMixture::expand_n_levels(int n_levels) {
        VLOG(10) << "Expanding hierarchy of n levels...";

        run_em();
        std::cout << "Done with em";

        int n_nodes_to_create = std::pow(AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE, n_levels);
        std::vector<DeviceHierarchicalGaussianMixture*> to_expand;
        std::cout << "Adding root to queue";
        to_expand.push_back(this);

        while(n_nodes_to_create > 0) {

            n_nodes_to_create -= AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE;
        }

        VLOG(10) << "Done expanding hierarchy of n levels.";
    }

    void DeviceHierarchicalGaussianMixture::run_em() {
        auto ptr = std::shared_ptr<DevicePointCloud>(new DevicePointCloud(pcl));
        PointCloud vanilla_pcl(ptr);

        auto em = EmAlgorithm::from_pcl(vanilla_pcl);
        em.run(EM_CONVERGENCE_THRESHOLD);
    }

    DeviceHierarchicalGaussianMixture DeviceHierarchicalGaussianMixture::create_one_child(const DevicePointCloud::PointIterator& begin,
                                                                                          const DevicePointCloud::PointIterator& end,
                                                                                          const WeightedGaussian& parent_distribution) const {
        GaussianMixtureFactory gmm_factory;
        DevicePointCloud child_pcl;

        VLOG(1) << "N of points in child :" << end - begin;

        child_pcl.set_points(pcl.get_data(), begin, end);
        auto child_mixture = gmm_factory.around_point(parent_distribution.get_mu(),
                                                      parent_distribution.get_sigma(),
                                                      UNIFORM_DISTRIBUTION_SIZE,
                                                      AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE);

        return DeviceHierarchicalGaussianMixture(child_pcl, child_mixture);
    }
}
