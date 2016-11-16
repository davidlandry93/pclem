
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

    void DeviceHierarchicalGaussianMixture::create_children() {
        VLOG(10) << "Creating children of hierarchical gaussian mixture...";
        SortByBestAssociationOperation op;

        std::vector<DevicePointCloud::PointIterator> boundaries = pcl.execute_pointcloud_operation(op);

        //children.push_back(create_one_child(sub_pcl_begin, first_unsorted, mixture.get_gaussian(i)));

        VLOG(10) << "Done creating children";
    }

    void DeviceHierarchicalGaussianMixture::expand_n_levels(int n_levels) {
        
    }

    void DeviceHierarchicalGaussianMixture::run_em() {
        PointCloud vanilla_pcl(&pcl);

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
