#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>

namespace pclem {
    class DevicePointCloud;
    class PointCloud {
    public:
        PointCloud();
        void add_points(const std::vector<double>& values);

    private:
        std::unique_ptr<DevicePointCloud> device_pcl;
    };
}

#endif
