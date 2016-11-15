
#include <limits>

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
        double pos_inf = std::numeric_limits<double>::infinity();
        double neg_inf = -1*std::numeric_limits<double>::infinity();

        DevicePoint min = thrust::reduce(begin, end, DevicePoint(pos_inf, pos_inf, pos_inf), min_function);
        DevicePoint max = thrust::reduce(begin, end, DevicePoint(neg_inf, neg_inf, neg_inf), max_function);

        Point host_min = min.to_host();
        Point host_max = max.to_host();

        BoundingBox bounding_box(host_min, host_max);

        return bounding_box;
    }
}
