#ifndef HIERARCHICAL_GAUSSIAN_MIXTURE_H
#define HIERARCHICAL_GAUSSIAN_MIXTURE_H

#include <vector>
#include <memory>

#include "weighted_gaussian.h"
#include "visualization.h"

namespace pclem {
    class DeviceHierarchicalGaussianMixture;
    class HierarchicalGaussianMixture {
    public:
        HierarchicalGaussianMixture(const std::shared_ptr<DeviceHierarchicalGaussianMixture>& device_mixture);
        ~HierarchicalGaussianMixture();
        void insert_into_visualization(Visualization& vis) const;
        friend std::ostream& operator<<(std::ostream& os, const HierarchicalGaussianMixture& hierarchy);
    private:
        std::shared_ptr<DeviceHierarchicalGaussianMixture> device_mixture;
    };
}

#endif
