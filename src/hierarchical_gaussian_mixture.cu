
#include <memory>

#include "hierarchical_gaussian_mixture.h"
#include "device_hierarchical_gaussian_mixture.h"

namespace pclem {
    HierarchicalGaussianMixture::HierarchicalGaussianMixture(const std::shared_ptr<DeviceHierarchicalGaussianMixture>& device_mixture) :
        device_mixture(device_mixture) {}

    HierarchicalGaussianMixture::~HierarchicalGaussianMixture() {
    }
}
