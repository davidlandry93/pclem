
#include "pointcloud.h"

namespace pclem {

    PointCloud::PointCloud(std::vector<double> data, int npoints) :
        n_of_points(npoints) {
        data = std::move(data);
    }

    PointCloud PointCloud::from_vtk(vtkPolyData* vtkData) {
        vtkIdType npoints = vtkData->GetNumberOfPoints();
        vtkPoints* points = vtkData->GetPoints();

        auto pointData = std::vector<double>();
        for(int i=0; i < npoints; i++) {
            double currentPoint[3];

            points->GetPoint(i, currentPoint);
            pointData.push_back(currentPoint[0]);
            pointData.push_back(currentPoint[1]);
            pointData.push_back(currentPoint[2]);
        }

        return PointCloud(pointData, npoints);
    }
}
