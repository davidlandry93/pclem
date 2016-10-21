
#include <iostream>
#include <limits>
#include <cmath>

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
    double previous_likelihood = 0.0;
    double delta = std::numeric_limits<double>::infinity();
    while (delta > 0.0001){
        em.expectation();
        em.maximization();

        double new_likelihood = em.log_likelihood();
        delta = std::abs(new_likelihood - previous_likelihood);
        std::cout << ". LOG LIKELIHOOD: " << new_likelihood << " Delta: " << delta << std::endl;

        previous_likelihood = new_likelihood;
    }
    std::cout << em;

    return 0;
}
