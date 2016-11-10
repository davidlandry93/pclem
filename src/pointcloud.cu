
#include <glog/logging.h>
#include <thrust/device_vector.h>

#include "associated_point.cuh"
#include "pointcloud.h"
#include "device_pointcloud.h"

namespace pclem {
    PointCloud::PointCloud() : device_pcl(new DevicePointCloud()) {}

    PointCloud::~PointCloud() {
        if(device_pcl != NULL) {
            delete device_pcl;
        }
    }

    PointCloud::PointCloud(PointCloud& other)
    {
        if(other.device_pcl == NULL) {
            device_pcl = new DevicePointCloud();
        } else {
            device_pcl = new DevicePointCloud(*other.device_pcl);
        }
    }

    PointCloud::PointCloud(PointCloud&& other) {
        device_pcl = other.device_pcl;
        other.device_pcl = NULL;
    }

    PointCloud& PointCloud::operator=(PointCloud&& other) {
        device_pcl = other.device_pcl;
        other.device_pcl = NULL;
        return *this;
    }

    void PointCloud::set_points(const std::vector<Point>& points) {
        VLOG(10) << "Setting points...";

        std::shared_ptr<thrust::device_vector<AssociatedPoint>> device_vector(new thrust::device_vector<AssociatedPoint>());

        for(Point point : points) {
            AssociatedPoint device_point(point);
            device_vector->push_back(device_point);
        }

        device_pcl->set_points(device_vector);

        VLOG(10) << "Done setting points.";
    }

    BoundingBox PointCloud::getBoundingBox() const {
        return device_pcl->getBoundingBox();
    }

    void PointCloud::compute_associations(const GaussianMixture& mixture) {
        device_pcl->compute_associations(mixture);
    }

    void PointCloud::normalize_associations() {
        device_pcl->normalize_associations();
    }

    GaussianMixture PointCloud::create_mixture() const {
        return device_pcl->create_mixture();
    }

    double PointCloud::log_likelihood_of_mixture(const GaussianMixture& mixture) const {
        return device_pcl->log_likelihood_of_mixture(mixture);
    }

    std::vector<Point> PointCloud::copy_of_points() const {
        return device_pcl->copy_of_points();
    }

    HierarchicalGaussianMixture PointCloud::create_hgmm() const {
        return device_pcl->create_hgmm();
    }

}
