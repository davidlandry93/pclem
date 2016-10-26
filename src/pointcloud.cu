
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

    PointCloud::PointCloud(PointCloud& other) {
        device_pcl = other.device_pcl;
        other.device_pcl = NULL;
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

    void PointCloud::add_points(const std::vector<Point>& points) {
        device_pcl->add_points(points);
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

}
