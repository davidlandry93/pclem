#include <iostream>

#include <glog/logging.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleJoystickCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkParametricFunction.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkProperty.h>
#include <vtkCamera.h>

#include <vtkTransform.h>

#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include "vtk_visualization.h"

namespace pclem {
    VtkVisualization::VtkVisualization() :
        points(vtkSmartPointer<vtkPoints>::New()),
        polydata(vtkSmartPointer<vtkPolyData>::New()),
        cells(vtkSmartPointer<vtkCellArray>::New()),
        renderer(vtkSmartPointer<vtkRenderer>::New()),
        interactor(vtkSmartPointer<vtkRenderWindowInteractor>::New()),
        axes_widget(vtkSmartPointer<vtkOrientationMarkerWidget>::New())
    {
        auto interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        interactor->SetInteractorStyle(interactorStyle);

        renderer->SetBackground(0.2,0.2,0.2);
    }

    void VtkVisualization::insert_point(const Point& point) {
        VLOG(15) << "Inserting point in visualization...";

        vtkIdType pointId = points->InsertNextPoint(point.x, point.y, point.z);
        cells->InsertNextCell(1);
        cells->InsertCellPoint(pointId);

        points->Modified();
        cells->Modified();

        VLOG(15) << "Done inserting point in visualization.";
    }

    double rad_2_deg(double rad) {
        const double ratio = 360.0 / (2 * M_PI);
        return rad * ratio;
    }

    void VtkVisualization::insert_ellipsoid(const Ellipsoid& ellipsoid) {
        auto vtk_ellipsoid = vtkSmartPointer<vtkParametricEllipsoid>::New();
        vtk_ellipsoid->SetXRadius(ellipsoid.a);
        vtk_ellipsoid->SetYRadius(ellipsoid.b);
        vtk_ellipsoid->SetZRadius(ellipsoid.c);

        auto function_source = vtkSmartPointer<vtkParametricFunctionSource>::New();
        function_source->SetParametricFunction(vtk_ellipsoid);
        function_source->SetUResolution(16);
        function_source->SetVResolution(16);
        function_source->SetWResolution(16);
        function_source->Update();

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(function_source->GetOutputPort());
        mapper->Update();

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        auto rotationMatrix = vtkSmartPointer<vtkMatrix4x4>::New(); 
        rotationMatrix->Identity();
        for(int i=0; i < 3; i++) {
            for(int j=0; j<3; j++) {
                rotationMatrix->SetElement(i,j, ellipsoid.rotation.get_element(i,j));
            }
        }

        auto transform = vtkSmartPointer<vtkTransform>::New();
        transform->SetMatrix(rotationMatrix);

        actor->SetUserTransform(transform);
        actor->SetPosition(ellipsoid.position.x, ellipsoid.position.y, ellipsoid.position.z);
        actor->GetProperty()->SetColor(1.0, 0.5, 0.0);
        actor->GetProperty()->SetOpacity(ellipsoid.opacity);

        renderer->AddActor(actor);
    }

    void VtkVisualization::insert_pointcloud() const {
        auto polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->SetPoints(points);
        polydata->SetVerts(cells);

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);
        mapper->Update();

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        renderer->AddActor(actor);
    }

    void VtkVisualization::insert_axes_widget() const {
        auto axes_actor = vtkSmartPointer<vtkAxesActor>::New();

        axes_widget->SetOrientationMarker(axes_actor);
        axes_widget->SetInteractor(interactor);
        axes_widget->SetEnabled(1);
        axes_widget->InteractiveOn();
    }

    void VtkVisualization::visualize() {
        VLOG(10) << "Visualizing...";

        auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->AddRenderer(renderer);

        interactor->SetRenderWindow(renderWindow);

        insert_pointcloud();
        insert_axes_widget();

        auto camera = vtkSmartPointer<vtkCamera>::New();
        camera->SetPosition(-25, -25, 5);
        camera->SetFocalPoint(0,0,0);
        renderer->SetActiveCamera(camera);

        renderWindow->SetSize(1000,1000);
        renderWindow->Render();
        interactor->Start();

        VLOG(10) << "Done visualizing.";
    }
}
