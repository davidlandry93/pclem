#ifndef VTK_VISUALIZATION_H
#define VTK_VISUALIZATION_H


#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkRenderer.h>

#include "visualization.h"
#include "point.h"
#include "ellipsoid.h"

namespace pclem {
    class VtkVisualization : public Visualization {
    public:
        VtkVisualization();
        void insert_point(const Point& point);
        void insert_ellipsoid(const Ellipsoid& ellipsoid);
        void visualize();
    private:
        vtkSmartPointer<vtkPoints> points;
        vtkSmartPointer<vtkPolyData> polydata;
        vtkSmartPointer<vtkCellArray> cells;

        vtkSmartPointer<vtkRenderer> renderer;
    };
}

#endif
