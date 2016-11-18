#ifndef MIXTURE_CREATION_OPERATION_H
#define MIXTURE_CREATION_OPERATION_H

#include "gaussian_mixture.h"
#include "covariance_matrix.h"
#include "weighted_gaussian.h"
#include "device_pointcloud.h"

namespace pclem {

    class MixtureCreationOperation : public DevicePointCloud::PointCloudOperation<GaussianMixture> {
    public:
        GaussianMixture operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) const;
    private:
        const double DROPOUT_WEIGHT = 1e-6;

        WeightedGaussian create_distribution_of_mixture(const DevicePointCloud::PointIterator& begin,
                                                        const DevicePointCloud::PointIterator& end,
                                                        int index_of_distribution, double sum_of_gammas) const;

        CovarianceMatrix compute_sigma(const DevicePointCloud::PointIterator& begin,
                                       const DevicePointCloud::PointIterator& end,
                                       int index_of_distribution, const Point& mu, double sum_of_gammas) const;
};

}

#endif
