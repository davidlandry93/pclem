#ifndef VISUALIZATION_H
#define VISUALIZATION_H


#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include "point.cuh"
#include "ellipsoid.h"

namespace pclem {
    class Visualization {
    public:
        Visualization();
        void insert_point(const Point& point);
        void insert_ellipsoid(const Ellipsoid& ellipsoid);
        void visualize();
    private:
        vtkSmartPointer<vtkPoints> points;
    };
}

#endif
