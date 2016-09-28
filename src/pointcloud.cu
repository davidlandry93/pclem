
#include <chrono>
#include <limits>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>

#include "pointcloud.h"

using namespace std::chrono;

namespace pclem {

    PointCloud::PointCloud(std::vector<double> data, int npoints) :
        data(data), n_of_points(npoints) {
        updateBoundingBox();
    }

    PointCloud::PointCloud(PointCloud&& other) :
        n_of_points(other.n_of_points), boundingBox(other.boundingBox) {
        data = std::move(data);
    }

    PointCloud& PointCloud::operator=(PointCloud&& other) {
        std::swap(data, other.data);
        return *this;
    }

    PointCloud PointCloud::from_vtk(vtkPolyData* vtkData) {
        vtkIdType npoints = vtkData->GetNumberOfPoints();
        vtkPoints* points = vtkData->GetPoints();

        auto stl_vec = std::vector<double>(3*npoints);
        for(int i=0; i < npoints; i++) {
            double currentPoint[3];

            points->GetPoint(i, currentPoint);
            stl_vec[i] = currentPoint[0];
            stl_vec[i + npoints] = currentPoint[1];
            stl_vec[i + 2*npoints] = currentPoint[2];
        }

        return PointCloud(stl_vec, npoints);
    }

    BoundingBox PointCloud::getBoundingBox() {
        return boundingBox;
    }

    void PointCloud::updateBoundingBox(){
        double min[3];
        double max[3];

        for(int i=0; i < 3; i++) {
            auto found_max = thrust::max_element(data.begin() + i*n_of_points,
                                                 data.begin() + (i + 1)*n_of_points);
            auto found_min = thrust::min_element(data.begin() + i*n_of_points,
                                                 data.begin() + (i + 1)*n_of_points);
            max[i] = *found_max;
            min[i] = *found_min;
        }

        boundingBox.setMin(Point(min[0], min[1], min[2]));
        boundingBox.setMax(Point(max[0], max[1], max[2]));
    }
}
