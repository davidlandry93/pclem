
#include "device_hierarchical_gaussian_mixture.h"

namespace pclem {

    DeviceHierarchicalGaussianMixture::DeviceHierarchicalGaussianMixture(const thrust::device_vector<AssociatedPoint>::iterator& pcl_begin,
                                                                         const thrust::device_vector<AssociatedPoint>::iterator& pcl_end,
                                                                         GaussianMixture& mixture) :
        pcl_begin(pcl_begin), pcl_end(pcl_end), mixture(std::move(mixture)) {
        
    }
    
    void DeviceHierarchicalGaussianMixture::create_children() {
        
    }
}
