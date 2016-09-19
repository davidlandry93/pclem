
#include <iostream>

#include <vtkPointSet.h>
#include <vtkSmartPointer.h>
#include <vtkSimplePointsReader.h>
#include <vtkGenericDataObjectReader.h>

int main(int argc, char** argv) {
    std::cout << "Hello world" << std::endl;

    vtkSmartPointer<vtkGenericDataObjectReader> reader =
        vtkSmartPointer<vtkGenericDataObjectReader>::New();

    reader->SetFileName("res/bunny.vtk");
    reader->Update();

    vtkDataObject* data = reader->GetOutput();
    vtkDataSetAttributes* attributes = data->GetAttributes(vtkDataObject::POINT);

    std::cout << "output has " << data->GetDataObjectType() << " points." << std::endl;

    return 0;
}
