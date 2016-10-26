
#include "associated_point.cuh"
#include "pointcloud.h"
#include "device_pointcloud.h"

namespace pclem {
    PointCloud::PointCloud() : device_pcl(new DevicePointCloud()) {}

    void PointCloud::add_points(const std::vector<double>& values) {
        std::vector<AssociatedPoint> points;
        for(int i=0; i < values.size(); i += 3) {
            points.push_back(AssociatedPoint(values[i], values[i+1], values[i+2]));
        }
        device_pcl->add_points(points);
    }
}
