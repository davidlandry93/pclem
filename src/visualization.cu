#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL,vtkRenderingOpenGL)
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)

#include <iostream>

#include <glog/logging.h>

#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include "visualization.h"

namespace pclem {
    Visualization::Visualization() :
        points(vtkSmartPointer<vtkPoints>::New()),
        polydata(vtkSmartPointer<vtkPolyData>::New()) {}

    void Visualization::insert_point(const Point& point) {
        VLOG(15) << "Inserting point in visualization...";

        points->InsertNextPoint(point.x, point.y, point.z);

        VLOG(15) << "Done inserting point in visualization.";
    }

    void Visualization::insert_ellipsoid(const Ellipsoid& ellipsoid) {
        
    }

    void Visualization::visualize() {
        VLOG(10) << "Visualizing...";

        polydata->SetPoints(points);

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->SetBackground(1,.5,1);

        auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->AddRenderer(renderer);

        auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        renderWindowInteractor->SetRenderWindow(renderWindow);

        renderer->AddActor(actor);

        renderWindow->Render();
        renderWindowInteractor->Start();

        VLOG(10) << "Done visualizing.";
    }
}
