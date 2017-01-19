
#include <gtest/gtest.h>

#include "pointcloud.h"
#include "hierarchical_gaussian_mixture.h"
#include "vtk_pointcloud_reader.h"

namespace pclem {
    TEST(HierarchicalGaussianMixtureTest, Build2levels) {
        PointCloud pointcloud = VtkPointCloudReader::read("example.vtk");

        auto hgmm = pointcloud.create_hgmm(2);

        double likelihood = hgmm.log_likelihood_of_pointcloud(pointcloud);

        std::cout << likelihood << std::endl;
    }
}
