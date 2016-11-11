#ifndef GAUSSIAN_MIXTURE_FACTORY_H
#define GAUSSIAN_MIXTURE_FACTORY_H

#include "pointcloud.h"
#include "device_pointcloud.h"

namespace pclem {
    class GaussianMixtureFactory {
    public:
        GaussianMixtureFactory();
        GaussianMixture from_pcl_corners(const PointCloud& pcl) const;
        GaussianMixture from_pcl_corners(const DevicePointCloud& pcl) const;

        GaussianMixture around_point(const Point& point, const CovarianceMatrix& cov,
                                     int n_of_distributions, double delta) const;
    private:
        static bool random_seeded;

        GaussianMixture from_pcl_corners(const BoundingBox& bounding_box) const;
        static double random_number(double min, double max);
    };
}

#endif
