#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>

#include "point.h"
#include "boundingbox.h"
#include "gaussian_mixture.h"

namespace pclem {
    class DevicePointCloud;
    class PointCloud {
    public:
        PointCloud();
        ~PointCloud();
        PointCloud(PointCloud&& other);
        PointCloud& operator=(PointCloud&& other);
        void operator=(const PointCloud& other) = delete;

        void add_points(const std::vector<Point>& points);
        BoundingBox getBoundingBox() const;
        void compute_associations(const GaussianMixture& mixture);
        void normalize_associations();
        GaussianMixture create_mixture() const;
        double log_likelihood_of_mixture(const GaussianMixture& mixture) const;

    private:
        PointCloud(PointCloud& other);
        DevicePointCloud* device_pcl;
    };
}

#endif
