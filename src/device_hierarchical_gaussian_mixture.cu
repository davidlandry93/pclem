
#include "device_hierarchical_gaussian_mixture.h"

namespace pclem {
    DeviceHierarchicalGaussianMixture::DeviceHierarchicalGaussianMixture(thrust::device_vector<AssociatedPoint>& points)
        : pcl_begin(points.begin()), pcl_end(points.end()), mixture(std::move(mixture)) {
    }
}
