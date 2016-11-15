#ifndef LOG_LIKELIHOOD_OPERATION_H
#define LOG_LIKELIHOOD_OPERATION_H

#include "device_pointcloud.h"

namespace pclem {
    class LogLikelihoodOperation : public DevicePointCloud::PointCloudOperation<double> {
    public:
        LogLikelihoodOperation(const GaussianMixture& mixture);
        double operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) const;
    private:
        GaussianMixture mixture;

        double log_likelihood_of_distribution(const DevicePointCloud::PointIterator& begin,
                                              const DevicePointCloud::PointIterator& end,
                                              int index_of_distribution, const WeightedGaussian& distribution) const;
    };
}

#endif
