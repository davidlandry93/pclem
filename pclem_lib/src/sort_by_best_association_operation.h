#ifndef SORT_BY_BEST_ASSOCIATION_OPERATION_H
#define SORT_BY_BEST_ASSOCIATION_OPERATION_H

#include <vector>
#include "device_pointcloud.h"

namespace pclem {
    class SortByBestAssociationOperation : public DevicePointCloud::PointCloudOperation<std::vector<DevicePointCloud::PointIterator>> {
    public:
        std::vector<DevicePointCloud::PointIterator> operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) const;
    private:
    };
}

#endif
