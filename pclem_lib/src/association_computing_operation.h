
#ifndef ASSOCIATION_COMPUTING_OPERATION_H
#define ASSOCIATION_COMPUTING_OPERATION_H

#include "device_pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class AssociationComputingOperation : public DevicePointCloud::PointCloudOperation<void> {
    public:
        AssociationComputingOperation(const GaussianMixture& mixture, const double& volume);
        void operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end);
    private:
        const int UNIFORM_DISTRIBUTION_ID = 8; // In the associated points, the 9th association is a uniform distribution.

        GaussianMixture mixture;
        double volume_of_pcl;

        void select_and_execute_op(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end, const int& i);
    };
}

#endif
