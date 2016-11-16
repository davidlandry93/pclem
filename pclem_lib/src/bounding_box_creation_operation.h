#ifndef BOUNDING_BOX_CREATION_OPERATION_H
#define BOUNDING_BOX_CREATION_OPERATION_H

#include "device_pointcloud.h"
#include "boundingbox.h"

namespace pclem {
    class BoundingBoxCreationOperation : public DevicePointCloud::PointCloudOperation<BoundingBox> {
    public:
        BoundingBox operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) const;

    };
}

#endif
