
#include <string>
#include <iostream>
#include <limits>
#include <cmath>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "pointcloud.h"
#include "gaussian_mixture.h"
#include "weighted_gaussian.h"
#include "vtk_pointcloud_reader.h"
#include "vtk_visualization.h"
#include "file_exporter.h"

DEFINE_bool(vtk, false, "Display the hierarchy using vtk.");
DEFINE_string(input, "", "Path to the vtk file to load.");
DEFINE_string(output, "", "Where to store the output csv file");
DEFINE_uint32(n_points, std::numeric_limits<uint32_t>::max(), "Take the first n points from the point cloud.");
DEFINE_uint32(n_levels, 2, "The number of levels to expand in the hierarchy.");

using namespace pclem;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::string usage("Run hierarchical expectation-maximization on a point cloud.\n");
    usage += argv[0];
    usage +=  + " -input pcl.vtk -output out.csv -vtk";

    if(FLAGS_input.empty()) {
        std::cout << "No input pointcloud provided." << std::endl;
        return 0;
    }

    PointCloud pcl = VtkPointCloudReader::read(FLAGS_input, FLAGS_n_points);

    auto hgmm = pcl.create_hgmm(FLAGS_n_levels);

    if(!FLAGS_output.empty()) {
        FileExporter file_exporter(FLAGS_output);
        file_exporter.open_file();

        hgmm.insert_into_visualization(file_exporter);

        file_exporter.close();
    }

    if(FLAGS_vtk) {
        VtkVisualization vis;

        pcl.insert_into_visualization(vis);
        hgmm.insert_into_visualization(vis);

        vis.visualize();
    }

    return 0;
}
