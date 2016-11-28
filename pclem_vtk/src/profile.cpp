
#include <iostream>
#include <limits>
#include <cmath>

#include <glog/logging.h>

#include "em_algorithm.h"
#include "visualizable_point_cloud.h"
#include "visualizable_gaussian_mixture.h"
#include "vtk_pointcloud_reader.h"
#include "visualization.h"
#include "visualizable_weighted_gaussian.h"

using namespace pclem;

static int N_POINTS_TO_PROFILE = 2000;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    VisualizablePointCloud pcl = VtkPointCloudReader::read("res/foret.vtk", N_POINTS_TO_PROFILE);

    HierarchicalGaussianMixture hgmm = pcl.create_hgmm();

    std::vector<WeightedGaussian> leaves;
    hgmm.get_leaves(leaves);

    Visualization vis;
    pcl.insert_in_visualization(vis);
    std::cout << leaves.size() << " leaves to show." << std::endl;
    for(const WeightedGaussian& leave : leaves) {
        VisualizableWeightedGaussian visualizable_gaussian(leave);
        visualizable_gaussian.insert_into_visualization(vis);
    }

    vis.visualize();

    return 0;
}
