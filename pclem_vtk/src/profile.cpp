
#include <iostream>
#include <limits>
#include <cmath>

#include <glog/logging.h>

#include <pclem/em_algorithm.h>
#include <pclem/pointcloud.h>

#include <pclem/hierarchical_gaussian_mixture.h>

#include "visualizable_point_cloud.h"
#include "visualizable_gaussian_mixture.h"
#include "vtk_pointcloud_reader.h"
#include "visualization.h"

using namespace pclem;

static int N_POINTS_TO_PROFILE = 2000;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    VisualizablePointCloud pcl = VtkPointCloudReader::read("res/foret.vtk", N_POINTS_TO_PROFILE);

    auto hgmm = pcl.create_hgmm();

    return 0;
}
