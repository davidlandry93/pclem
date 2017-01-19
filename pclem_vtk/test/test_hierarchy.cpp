
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "vtk_pointcloud_reader.h"

namespace pclem {
    TEST(HierarchicalGaussianMixtureModelTest, BuildHierarchyTest) {
        auto pointcloud = VtkPointCloudReader::read("example.vtk", 1000000);
        auto hgmm = pointcloud.create_hgmm(4, 1.0);

        double log_lik = hgmm.log_likelihood_of_pointcloud(pointcloud);
        std::cout << log_lik;
    }
}
