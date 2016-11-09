#ifndef GAUSSIAN_MIXTURE_FACTORY_H
#define GAUSSIAN_MIXTURE_FACTORY_H

#include "pointcloud.h"
#include "device_pointcloud.h"

namespace pclem {
    class GaussianMixtureFactory {
    public:
        GaussianMixture from_pcl_corners(const PointCloud& pcl) const;
        GaussianMixture from_pcl_corners(const DevicePointCloud& pcl) const;
    private:
        GaussianMixture from_pcl_corners(const BoundingBox& bounding_box) const;
    };
}

#endif
