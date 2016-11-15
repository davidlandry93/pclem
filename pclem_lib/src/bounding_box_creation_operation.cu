
#include "bounding_box_creation_operation.h"

namespace pclem {
    struct min_op: public thrust::binary_function<DevicePoint,DevicePoint,DevicePoint> {
        __device__
        DevicePoint operator()(DevicePoint lhs, DevicePoint rhs) {
            return DevicePoint(thrust::min(lhs.x, rhs.x),
                               thrust::min(lhs.y, rhs.y),
                               thrust::min(lhs.z, rhs.z));
        }
    };

    struct max_op: public thrust::binary_function<DevicePoint,DevicePoint,DevicePoint> {
        __device__
        DevicePoint operator()(DevicePoint lhs, DevicePoint rhs) {
            return DevicePoint(thrust::max(lhs.x, rhs.x),
                               thrust::max(lhs.y, rhs.y),
                               thrust::max(lhs.z, rhs.z));
        }
    };

    BoundingBox BoundingBoxCreationOperation::operator()(const DevicePointCloud::PointIterator& begin,
                                                         const DevicePointCloud::PointIterator& end) {
        min_op min_function;
        max_op max_function;

        DevicePoint min = thrust::reduce(begin, end, DevicePoint(0.0,0.0,0.0), min_function);
        DevicePoint max = thrust::reduce(begin, end, DevicePoint(0.0,0.0,0.0), max_function);

        Point host_min = min.to_host();
        Point host_max = max.to_host();

        BoundingBox bounding_box(host_min, host_max);

        return bounding_box;
    }
}
