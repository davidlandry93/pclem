#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>
#include <vtkPolyData.h>

#include <thrust/device_vector.h>

#include "boundingbox.h"

namespace pclem {

    class PointCloud {
    public:
        static PointCloud from_vtk(vtkPolyData* vtkData);
        PointCloud(PointCloud&& other);
        PointCloud& operator=(PointCloud&& other);
        BoundingBox getBoundingBox();
    private:
        thrust::device_vector<double> data;
        int n_of_points;
        BoundingBox boundingBox;

        PointCloud(std::vector<double> data, int n_of_points);
        PointCloud(PointCloud& other);
        void updateBoundingBox();
    };

}

#endif
