
#include <glog/logging.h>
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
            return p.best_distribution == index_of_distribution;
        }
    private:
        __const__ int index_of_distribution;
    };

    std::vector<int> SortByBestAssociationOperation::operator()(const DevicePointCloud::PointIterator& begin,
                                                                const DevicePointCloud::PointIterator& end) const {

        VLOG(10) << "Sorting by best association...";

        std::vector<int> boundaries;
        DevicePointCloud::PointIterator first_unsorted = begin;

        boundaries.push_back(begin - begin);
        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            VLOG(11) << "Sorting distribution " << i;
            is_most_associated_op op(i);

            thrust::partition(first_unsorted, end, op);
            first_unsorted = thrust::find_if_not(first_unsorted, end, op);

            VLOG(11) << "Storing result";
            boundaries.push_back(first_unsorted - begin);
        }

        VLOG(10) << "Done sorting by best association...";
        return boundaries;
    }

}
