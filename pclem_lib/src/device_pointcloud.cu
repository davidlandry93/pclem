
#include <glog/logging.h>

#include <thrust/device_vector.h>

#include "device_pointcloud.h"
#include "gaussian_mixture_factory.h"
#include "device_hierarchical_gaussian_mixture.h"
#include "em_algorithm.h"

#include "association_computing_operation.h"
#include "point_normalization_operation.h"
#include "log_likelihood_operation.h"
#include "mixture_creation_operation.h"
#include "bounding_box_creation_operation.h"

namespace pclem {
    DevicePointCloud::DevicePointCloud() :
        ptr_to_points(new thrust::device_vector<AssociatedPoint>()),
        pts_begin(ptr_to_points->begin()),
        pts_end(ptr_to_points->end()),
        boundingBox() {}

    DevicePointCloud::DevicePointCloud(const DevicePointCloud& other) :
        ptr_to_points(other.ptr_to_points),
        pts_begin(other.pts_begin),
        pts_end(other.pts_end),
        boundingBox(other.boundingBox){}

    BoundingBox DevicePointCloud::getBoundingBox() const {
        return BoundingBox(boundingBox);
    }

    void DevicePointCloud::updateBoundingBox(){
        VLOG(10) << "Updating bounding box...";

        BoundingBoxCreationOperation op;

        boundingBox = execute_pointcloud_operation(op);

        VLOG(10) << "Done updating bounding box.";
    }

    int DevicePointCloud::get_n_points() const {
        return pts_end - pts_begin;
    }

    void DevicePointCloud::set_points(const std::shared_ptr<thrust::device_vector<AssociatedPoint>>& points) {

        set_points(points, points->begin(), points->end());
    }

    void DevicePointCloud::set_points(const std::shared_ptr<thrust::device_vector<AssociatedPoint>>& points,
                                      const PointIterator& begin,
                                      const PointIterator& end) {
        VLOG(10) << "Setting new data source...";

        ptr_to_points = points;
        pts_begin = begin;
        pts_end = end;

        updateBoundingBox();

        VLOG(10) << "Done setting data source.";
    }

    void DevicePointCloud::compute_associations(const GaussianMixture& mixture) {
        VLOG(10) << "Computing point/distribution associations...";

        AssociationComputingOperation op(mixture);
        execute_pointcloud_operation(op);

        VLOG(10) << "Done computing point/distribution associations.";
    }

    void DevicePointCloud::normalize_associations() {
        execute_pointcloud_operation(PointNormalizationOperation());
    }

    GaussianMixture DevicePointCloud::create_mixture() const {
        return execute_pointcloud_operation(MixtureCreationOperation());
    }

    double DevicePointCloud::log_likelihood_of_mixture(const GaussianMixture& mixture) const {
        LogLikelihoodOperation op(mixture);
        return execute_pointcloud_operation<double>(op);
    }

    std::vector<Point> DevicePointCloud::copy_of_points() const {
        std::vector<Point> copy;

        for(auto it = pts_begin; it != pts_end; it++) {
            AssociatedPoint device_point(*it);
            copy.push_back(device_point.to_host());
        }

        return copy;
    }

    HierarchicalGaussianMixture DevicePointCloud::create_hgmm() {
        VLOG(10) << "Creating hgmm...";
        GaussianMixtureFactory gmm_factory;

        std::shared_ptr<DeviceHierarchicalGaussianMixture> hierarchical_mixture(new DeviceHierarchicalGaussianMixture(*this, gmm_factory.from_pcl_corners(*this), WeightedGaussian()));
        hierarchical_mixture->expand_n_levels(2);

        VLOG(10) << "Done creating hgmm.";
        return HierarchicalGaussianMixture(hierarchical_mixture);
    }

    DevicePointCloud::PointIterator DevicePointCloud::begin() {
        return pts_begin;
    }

    DevicePointCloud::PointIterator DevicePointCloud::end() {
        return pts_end;
    }

    std::shared_ptr<thrust::device_vector<AssociatedPoint>> DevicePointCloud::get_data() const {
        return ptr_to_points;
    }
}
