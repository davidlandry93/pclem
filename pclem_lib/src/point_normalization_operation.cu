
#include "point_normalization_operation.h"
namespace pclem {

    struct normalization_op : public thrust::unary_function<AssociatedPoint, AssociatedPoint> {
        __host__ __device__
        AssociatedPoint operator()(AssociatedPoint p) {
            double sum_of_associations = 0.0;
            for(int i = 0; i < p.N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                sum_of_associations += p.associations[i];
            }

            for(int i = 0; i < p.N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                p.associations[i] = p.associations[i] / sum_of_associations;
            }

            return p;
        }
    };

    void PointNormalizationOperation::operator()(DevicePointCloud::PointIterator begin, DevicePointCloud::PointIterator end) const {
        thrust::transform(begin, end, begin, normalization_op());
    }
}
