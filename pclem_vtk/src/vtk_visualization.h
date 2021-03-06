#ifndef VTK_VISUALIZATION_H
#define VTK_VISUALIZATION_H


#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOrientationMarkerWidget.h>

#include "visualization.h"
#include "point.h"
#include "ellipsoid.h"
#include "parula_lookup_table.h"

namespace pclem {
    class VtkVisualization : public Visualization {
    public:
        VtkVisualization();
        void insert_point(const Point& point);
        void insert_ellipsoid(const Ellipsoid& ellipsoid);
        void visualize();
    private:
        vtkSmartPointer<vtkPoints> points;
        vtkSmartPointer<vtkCellArray> cells;
        vtkSmartPointer<vtkRenderer> renderer;
        vtkSmartPointer<vtkRenderWindowInteractor> interactor;
        vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget;
        ParulaLookupTable color_table;

        void insert_axes_widget() const;
        void insert_pointcloud() const;
        void insert_axes() const;
    };
}

#endif
