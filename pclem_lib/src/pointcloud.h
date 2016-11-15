#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>

#include "point.h"
#include "boundingbox.h"
#include "gaussian_mixture.h"
#include "hierarchical_gaussian_mixture.h"

namespace pclem {
    class DevicePointCloud;
    class PointCloud {
    public:
        PointCloud();
        ~PointCloud();
        PointCloud(PointCloud& other);
        PointCloud(PointCloud&& other);
        PointCloud(DevicePointCloud* pcl_ptr);
        PointCloud& operator=(PointCloud&& other);
        void operator=(const PointCloud& other) = delete;

        void set_points(const std::vector<Point>& points);
        BoundingBox getBoundingBox() const;
        void compute_associations(const GaussianMixture& mixture);
        void normalize_associations();
        GaussianMixture create_mixture() const;
        double log_likelihood_of_mixture(const GaussianMixture& mixture) const;
        HierarchicalGaussianMixture create_hgmm() const;

        std::vector<Point> copy_of_points() const;

    private:
        DevicePointCloud* device_pcl;
    };
}

#endif
