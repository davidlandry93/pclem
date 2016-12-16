#ifndef GAUSSIAN_MIXTURE_FACTORY_H
#define GAUSSIAN_MIXTURE_FACTORY_H

#include "pointcloud.h"
#include "device_pointcloud.h"
#include "matrix33.h"

namespace pclem {
    class GaussianMixtureFactory {
    public:
        GaussianMixtureFactory();
        GaussianMixture from_pcl_corners(const PointCloud& pcl, double weight_of_parent_in_hierarchy) const;
        GaussianMixture from_pcl_corners(const DevicePointCloud& pcl, double weight_of_parent_in_hierarchy) const;

        GaussianMixture around_point(const Point& point, const Matrix33& cov,
                                     int n_of_distributions, double delta, double weight_of_parent_in_hierarchy) const;
    private:
        static bool random_seeded;

        GaussianMixture from_pcl_corners(const BoundingBox& bounding_box, double weight_of_parent_in_hierarchy) const;
        Matrix33 covariance_from_pcl_corners(const BoundingBox& bounding_box) const;
        static double random_number(double min, double max);
    };
}

#endif
