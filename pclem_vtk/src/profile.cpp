
#include <iostream>
#include <limits>
#include <cmath>

#include <glog/logging.h>

#include "em_algorithm.h"
#include "pointcloud.h"
#include "gaussian_mixture.h"
#include "vtk_pointcloud_reader.h"
#include "vtk_visualization.h"
#include "weighted_gaussian.h"
#include "file_exporter.h"

using namespace pclem;

static int N_POINTS_TO_PROFILE = 2000;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    PointCloud pcl = VtkPointCloudReader::read("res/foret.vtk", N_POINTS_TO_PROFILE);

    HierarchicalGaussianMixture hgmm = pcl.create_hgmm();

    std::vector<WeightedGaussian> leaves;
    hgmm.get_leaves(leaves);

    FileExporter file_exporter("output.csv");
    file_exporter.open_file();

    //pcl.insert_into_visualization(file_exporter);
    std::cout << leaves.size() << " leaves to show." << std::endl;
    for(const WeightedGaussian& leave : leaves) {
        leave.insert_into_visualization(file_exporter);
    }

    return 0;
}
