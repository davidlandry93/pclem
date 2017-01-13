#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>

#include "point.h"
#include "boundingbox.h"
#include "gaussian_mixture.h"
#include "visualization.h"
#include "hierarchical_gaussian_mixture.h"

namespace pclem {
    class DevicePointCloud;
    class PointCloud {
    public:
        PointCloud();
        PointCloud(PointCloud& other);
        PointCloud(PointCloud&& other);
        PointCloud(const std::shared_ptr<DevicePointCloud>& pcl_ptr);
        PointCloud& operator=(PointCloud&& other);
        void operator=(const PointCloud& other) = delete;

        void set_points(const std::vector<Point>& points);
        BoundingBox getBoundingBox() const;
        void compute_associations(const GaussianMixture& mixture);
        void normalize_associations();
        GaussianMixture create_mixture() const;
        double log_likelihood_of_mixture(const GaussianMixture& mixture) const;
        HierarchicalGaussianMixture create_hgmm(int n_levels) const;
        HierarchicalGaussianMixture create_hgmm(int n_levels, double em_convergence_threshold) const;
        int get_n_points() const;

        std::vector<Point> copy_of_points() const;

        void insert_into_visualization(Visualization& vis) const;

    private:
        std::shared_ptr<DevicePointCloud> device_pcl;
    };
}

#endif
