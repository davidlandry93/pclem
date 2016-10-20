
#include <iostream>

#include <vtkSmartPointer.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkPolyData.h>
#include <glog/logging.h>

#include "pointcloud.h"
#include "em_algorithm.h"

using namespace pclem;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Hello world";

    VLOG(10) << "Reading point cloud...";
    vtkSmartPointer<vtkGenericDataObjectReader> reader =
        vtkSmartPointer<vtkGenericDataObjectReader>::New();
    reader->SetFileName("res/foret.vtk");
    reader->Update();
    VLOG(10) << "Done.";

    vtkPolyData* output = reader->GetPolyDataOutput();

    PointCloud pcl = PointCloud::from_vtk(output);

    auto em = EmAlgorithm::from_pcl(pcl);

    std::cout << em;
    for(int i=0; i < 1; i++) {
        em.expectation();
        em.maximization();

        std::cout << i << ". LOG LIKELIHOOD: " << em.log_likelihood() << std::endl;
    }

    return 0;
}
