
#include <iostream>
#include <limits>
#include <cmath>

#include <glog/logging.h>

#include "em_algorithm.h"
#include "visualizable_point_cloud.h"
#include "visualizable_gaussian_mixture.h"
#include "vtk_pointcloud_reader.h"
#include "visualization.h"

using namespace pclem;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    VisualizablePointCloud pcl = VtkPointCloudReader::read("res/foret.vtk");

    auto em = EmAlgorithm::from_pcl(pcl);

    std::cout << em;

    Visualization vis;
    pcl.insert_in_visualization(vis);

    em.run(0.1);

    std::cout << em;

    VisualizableGaussianMixture mixture(em.get_mixture());
    mixture.insert_into_visualization(vis);

    vis.visualize();


    return 0;
}
