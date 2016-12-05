#ifndef DEVICE_POINTCLOUD_H
#define DEVICE_POINTCLOUD_H

#include <memory>
#include <vector>
#include <functional>

#include <thrust/device_vector.h>

#include "point.h"
#include "boundingbox.h"
#include "weighted_gaussian.h"
#include "gaussian_mixture.h"
#include "associated_point.cuh"
#include "hierarchical_gaussian_mixture.h"

namespace pclem {

    class DevicePointCloud {
    public:
        typedef thrust::device_vector<AssociatedPoint>::iterator PointIterator;

        template<typename T>
        using PointCloudOperation = std::function<T(const PointIterator&, const PointIterator&)>;

        DevicePointCloud();
        DevicePointCloud(const DevicePointCloud& other);
        BoundingBox getBoundingBox() const;
        int get_n_points() const;
        void compute_associations(const GaussianMixture& mixture);
        void normalize_associations();
        GaussianMixture create_mixture() const;
        double log_likelihood_of_mixture(const GaussianMixture& mixture) const;
        void set_points(const std::shared_ptr<thrust::device_vector<AssociatedPoint>>& points);
        void set_points(const std::shared_ptr<thrust::device_vector<AssociatedPoint>>& points,
                        const thrust::device_vector<AssociatedPoint>::iterator& begin,
                        const thrust::device_vector<AssociatedPoint>::iterator& end);
        std::vector<Point> copy_of_points() const;
        HierarchicalGaussianMixture create_hgmm();
        PointIterator begin();
        PointIterator end();
        std::shared_ptr<thrust::device_vector<AssociatedPoint>> get_data() const;

        template<typename T>
        T execute_pointcloud_operation(PointCloudOperation<T> op) const {
            return op(pts_begin, pts_end);
        }

        void insert_into_visualization(Visualization& vis) const;

    private:
        std::shared_ptr<thrust::device_vector<AssociatedPoint>> ptr_to_points;
        PointIterator pts_begin;
        PointIterator pts_end;
        BoundingBox boundingBox;

        DevicePointCloud(std::vector<AssociatedPoint> data);
        void updateBoundingBox();
    };

}

#endif
