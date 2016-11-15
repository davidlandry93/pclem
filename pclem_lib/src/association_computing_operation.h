
#ifndef ASSOCIATION_COMPUTING_OPERATION_H
#define ASSOCIATION_COMPUTING_OPERATION_H

#include "device_pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class AssociationComputingOperation : public DevicePointCloud::PointCloudOperation<void> {
    public:
        AssociationComputingOperation(const GaussianMixture& mixture);
        void operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end);
    private:
        GaussianMixture mixture;

        void compute_associations_of_distribution(const DevicePointCloud::PointIterator& begin,
                                                  const DevicePointCloud::PointIterator& end,
                                                  int index_of_distribution,
                                                  const WeightedGaussian& distribution);
    };
}

#endif
