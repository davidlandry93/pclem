
#include <iostream>
#include <limits>
#include <cmath>

#include <glog/logging.h>

#include "visualizable_point_cloud.h"
#include "vtk_pointcloud_reader.h"

using namespace pclem;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    VisualizablePointCloud pcl = VtkPointCloudReader::read("res/foret.vtk");

    auto hgmm = pcl.create_hgmm();

    return 0;
}
