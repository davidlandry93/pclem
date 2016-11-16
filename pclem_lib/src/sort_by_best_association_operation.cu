
#include <thrust/partition.h>

#include "associated_point.cuh"
#include "sort_by_best_association_operation.h"

namespace pclem {

    struct is_most_associated_op : public thrust::unary_function<AssociatedPoint,bool> {
    public:
        is_most_associated_op(int index_of_distribution) :
            index_of_distribution(index_of_distribution) {}

        __host__ __device__
        bool operator()(AssociatedPoint p) {
            bool is_most_associated = true;
            for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                if(p.likelihoods[i] > p.likelihoods[index_of_distribution]) {
                    is_most_associated = false;
                }
            }
            return is_most_associated;
        }
    private:
        __const__ int index_of_distribution;
    };

    std::vector<int> SortByBestAssociationOperation::operator()(const DevicePointCloud::PointIterator& begin,
                                                                const DevicePointCloud::PointIterator& end) const {

        std::vector<int> boundaries;
        thrust::device_vector<AssociatedPoint>::iterator first_unsorted = begin;

        boundaries.push_back(first_unsorted - begin);
        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            is_most_associated_op op(i);

            thrust::partition(first_unsorted, end, op);
            first_unsorted = thrust::find_if_not(first_unsorted, end, op);

            boundaries.push_back(first_unsorted - begin);
        }

        return boundaries;
    }

}
