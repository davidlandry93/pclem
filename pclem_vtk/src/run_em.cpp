
#include <iostream>
#include <limits>
#include <cmath>

#include <glog/logging.h>

#include "em_algorithm.h"
#include "pointcloud.h"
#include "gaussian_mixture.h"
#include "vtk_pointcloud_reader.h"
#include "vtk_visualization.h"

using namespace pclem;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    PointCloud pcl = VtkPointCloudReader::read("res/foret.vtk");

    auto em = EmAlgorithm::from_pcl(pcl);

    std::cout << em;

    VtkVisualization vis;
    pcl.insert_into_visualization(vis);

    em.run(0.1);

    std::cout << em;

    GaussianMixture mixture(em.get_mixture());
    mixture.insert_into_visualization(vis);

    vis.visualize();


    return 0;
}
