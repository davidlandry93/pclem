#ifndef MIXTURE_CREATION_OPERATION_H
#define MIXTURE_CREATION_OPERATION_H

#include "gaussian_mixture.h"
#include "weighted_gaussian.h"
#include "device_pointcloud.h"

namespace pclem {

    class MixtureCreationOperation : public DevicePointCloud::PointCloudOperation<GaussianMixture> {
    public:
        MixtureCreationOperation(double weight_of_parent_in_hierarchy);
        GaussianMixture operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) const;
    private:
        const double DROPOUT_WEIGHT = 0.1;

        double weight_of_parent_in_hierarchy;

        WeightedGaussian create_distribution_of_mixture(const DevicePointCloud::PointIterator& begin,
                                                        const DevicePointCloud::PointIterator& end,
                                                        int index_of_distribution, double sum_of_gammas) const;

        Matrix33 compute_sigma(const DevicePointCloud::PointIterator& begin,
                               const DevicePointCloud::PointIterator& end,
                               int index_of_distribution, const Point& mu, double sum_of_gammas) const;
};

}

#endif
