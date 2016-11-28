#include <iostream>

#include <glog/logging.h>

#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkParametricFunction.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkProperty.h>

#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include "visualization.h"

namespace pclem {
    Visualization::Visualization() :
        points(vtkSmartPointer<vtkPoints>::New()),
        polydata(vtkSmartPointer<vtkPolyData>::New()),
        cells(vtkSmartPointer<vtkCellArray>::New()),
        renderer(vtkSmartPointer<vtkRenderer>::New())
    {
        double viewport[4] = {-10.0, -10.0, 10.0, 10.0};
        renderer->SetBackground(0.4,1,1);
        renderer->SetViewport(viewport);
    }

    void Visualization::insert_point(const Point& point) {
        VLOG(10) << "Inserting point in visualization...";

        vtkIdType pointId = points->InsertNextPoint(point.x, point.y, point.z);
        cells->InsertNextCell(1);
        cells->InsertCellPoint(pointId);

        points->Modified();
        cells->Modified();

        VLOG(10) << "Done inserting point in visualization.";
    }

    double rad_2_deg(double rad) {
        const double ratio = 360.0 / (2 * M_PI);
        return rad * ratio;
    }

    void Visualization::insert_ellipsoid(const Ellipsoid& ellipsoid) {
        auto back_property = vtkSmartPointer<vtkProperty>::New();
        back_property->SetColor(1.0,0.0,0.0);

        auto vtk_ellipsoid = vtkSmartPointer<vtkParametricEllipsoid>::New();
        vtk_ellipsoid->SetXRadius(ellipsoid.a);
        vtk_ellipsoid->SetYRadius(ellipsoid.b);
        vtk_ellipsoid->SetZRadius(ellipsoid.c);

        auto function_source = vtkSmartPointer<vtkParametricFunctionSource>::New();
        function_source->SetParametricFunction(vtk_ellipsoid);
        function_source->SetUResolution(51);
        function_source->SetVResolution(51);
        function_source->SetWResolution(51);
        function_source->Update();

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(function_source->GetOutputPort());

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->SetBackfaceProperty(back_property);
        actor->SetPosition(ellipsoid.position.x, ellipsoid.position.y, ellipsoid.position.z);
        actor->RotateX(rad_2_deg(ellipsoid.rotation.x));
        actor->RotateY(rad_2_deg(ellipsoid.rotation.y));
        actor->RotateZ(rad_2_deg(ellipsoid.rotation.z));

        actor->GetProperty()->SetOpacity(0.5);

        renderer->AddActor(actor);
    }

    void Visualization::visualize() {
        VLOG(10) << "Visualizing...";

        polydata->SetPoints(points);
        polydata->SetVerts(cells);

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);
        mapper->Update();

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);


        auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->AddRenderer(renderer);

        auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        renderWindowInteractor->SetRenderWindow(renderWindow);

        auto axes_actor = vtkSmartPointer<vtkAxesActor>::New();
        auto widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();

        widget->SetOrientationMarker(axes_actor);
        widget->SetInteractor(renderWindowInteractor);
        widget->SetEnabled(1);
        widget->InteractiveOn();

        renderer->AddActor(actor);

        renderWindow->SetSize(800,600);
        renderWindow->Render();
        renderWindowInteractor->Start();

        VLOG(10) << "Done visualizing.";
    }
}
