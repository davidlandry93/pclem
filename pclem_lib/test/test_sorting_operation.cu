
#include "gtest/gtest.h"

#include <memory>
#include <thrust/device_vector.h>

#include "associated_point.cuh"
#include "device_pointcloud.h"
#include "sort_by_best_association_operation.h"

namespace pclem{
    TEST(EmptyPointcloudTest, ReturnsBegin) {
        thrust::device_vector<AssociatedPoint> vector;
        std::shared_ptr<thrust::device_vector<AssociatedPoint>> points(new thrust::device_vector<AssociatedPoint>); 
        DevicePointCloud pcl;
        pcl.set_points(points);

        SortByBestAssociationOperation op;
        auto partition = pcl.execute_pointcloud_operation(op);

        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            ASSERT_EQ(0, partition[i]);
        }
    }
}

