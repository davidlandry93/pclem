
#include <glog/logging.h>
#include <thrust/partition.h>
#include <thrust/find.h>

#include "associated_point.cuh"
#include "device_hierarchical_gaussian_mixture.h"
#include "pointcloud.h"

namespace pclem {
    DeviceHierarchicalGaussianMixture::DeviceHierarchicalGaussianMixture(const DevicePointCloud& pcl, const GaussianMixture& mixture)
        : pcl(pcl), mixture(mixture), children() {}

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

    void DeviceHierarchicalGaussianMixture::create_children() {
        VLOG(10) << "Creating children of hierarchical gaussian mixture...";

        thrust::device_vector<AssociatedPoint>::iterator first_unsorted = pcl.begin();
        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            is_most_associated_op op(i);
            DevicePointCloud::PointIterator sub_pcl_begin = first_unsorted;

            thrust::partition(first_unsorted, pcl.end(), op);
            first_unsorted = thrust::find_if_not(first_unsorted, pcl.end(), op);

            std::cout << first_unsorted - sub_pcl_begin << " ";
        }

        VLOG(10) << "Done creating children";
    }
}
