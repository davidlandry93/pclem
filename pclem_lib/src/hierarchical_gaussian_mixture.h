#ifndef HIERARCHICAL_GAUSSIAN_MIXTURE_H
#define HIERARCHICAL_GAUSSIAN_MIXTURE_H

#include <memory>

namespace pclem {
    class DeviceHierarchicalGaussianMixture;
    class HierarchicalGaussianMixture {
    public:
        HierarchicalGaussianMixture(const std::shared_ptr<DeviceHierarchicalGaussianMixture>& device_mixture);
        ~HierarchicalGaussianMixture();
    private:
        std::shared_ptr<DeviceHierarchicalGaussianMixture> device_mixture;
    };
}

#endif
