
#include <iostream>

#include <vtkSmartPointer.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkPolyData.h>

#include "pointcloud.h"

using namespace pclem;

int main(int argc, char** argv) {
    std::cout << "Hello world" << std::endl;

    vtkSmartPointer<vtkGenericDataObjectReader> reader =
        vtkSmartPointer<vtkGenericDataObjectReader>::New();
    reader->SetFileName("res/foret.vtk");
    reader->Update();

    vtkPolyData* output = reader->GetPolyDataOutput();

    PointCloud pcl = PointCloud::from_vtk(output);

    return 0;
}