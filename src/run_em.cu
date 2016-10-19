
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

    vtkSmartPointer<vtkGenericDataObjectReader> reader =
        vtkSmartPointer<vtkGenericDataObjectReader>::New();
    reader->SetFileName("res/foret.vtk");
    reader->Update();

    vtkPolyData* output = reader->GetPolyDataOutput();

    PointCloud pcl = PointCloud::from_vtk(output);

    auto em = EmAlgorithm::from_pcl(pcl);

    std::cout << em;
    for(int i=0; i < 100; i++) {
        em.expectation();
        em.maximization();
        if(i % 10 == 0) {
            std::cout << em;
        }

        std::cout << i << ". LOG LIKELIHOOD: " << em.log_likelihood() << std::endl;
    }

    return 0;
}
