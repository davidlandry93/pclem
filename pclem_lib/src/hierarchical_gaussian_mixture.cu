
#include <memory>

#include "hierarchical_gaussian_mixture.h"
#include "device_hierarchical_gaussian_mixture.h"

namespace pclem {
    HierarchicalGaussianMixture::HierarchicalGaussianMixture(const std::shared_ptr<DeviceHierarchicalGaussianMixture>& device_mixture)
        : device_mixture(device_mixture) {}

    HierarchicalGaussianMixture::~HierarchicalGaussianMixture() {
    }

    std::ostream& operator<<(std::ostream& os, const HierarchicalGaussianMixture& hierarchy) {
        os << *(hierarchy.device_mixture);
        return os;
    }

    void HierarchicalGaussianMixture::insert_into_visualization(Visualization& vis) const {
        device_mixture->insert_into_visualization(vis);
    }

    double HierarchicalGaussianMixture::log_likelihood_of_pointcloud(PointCloud& pointcloud) const {
        return device_mixture->log_likelihood_of_pointcloud(pointcloud);
    }
}
