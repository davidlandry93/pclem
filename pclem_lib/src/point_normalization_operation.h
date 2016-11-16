
#include "device_pointcloud.h"

namespace pclem{


class PointNormalizationOperation : public DevicePointCloud::PointCloudOperation<void> {
public:
    void operator()(DevicePointCloud::PointIterator begin, DevicePointCloud::PointIterator end) const;
};

}
