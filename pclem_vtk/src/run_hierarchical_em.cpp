
#include <iostream>
#include <limits>
#include <cmath>

#include <glog/logging.h>

#include "visualizable_point_cloud.h"
#include "visualizable_gaussian_mixture.h"
#include "visualizable_weighted_gaussian.h"
#include "vtk_pointcloud_reader.h"
#include "vtk_visualization.h"

using namespace pclem;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    VisualizablePointCloud pcl = VtkPointCloudReader::read("res/foret.vtk");

    auto hgmm = pcl.create_hgmm();

    std::vector<WeightedGaussian> leaves;
    hgmm.get_leaves(leaves);

    VtkVisualization vis;
    pcl.insert_in_visualization(vis);
    std::cout << leaves.size() << " leaves to show." << std::endl;
    for(const WeightedGaussian& leave : leaves) {
        VisualizableWeightedGaussian visualizable_gaussian(leave);
        visualizable_gaussian.insert_into_visualization(vis);
    }

    vis.visualize();

    return 0;
}
