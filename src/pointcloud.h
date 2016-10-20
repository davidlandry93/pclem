#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>
#include <vtkPolyData.h>

#include <thrust/device_vector.h>

#include "point.cuh"
#include "associated_point.cuh"
#include "boundingbox.h"

namespace pclem {

    class PointCloud {
    public:
        static PointCloud from_vtk(vtkPolyData* vtkData);
        PointCloud(PointCloud&& other);
        PointCloud& operator=(PointCloud&& other);
        BoundingBox getBoundingBox();
        int get_n_points() const;
        thrust::device_vector<Point>::const_iterator begin() const;
        thrust::device_vector<Point>::const_iterator end() const;
    private:
        thrust::device_vector<Point> data;
        int n_points;
        BoundingBox boundingBox;

        PointCloud(std::vector<Point> data, int n_of_points);
        PointCloud(PointCloud& other);
        void updateBoundingBox();
        void normalize_likelihoods(thrust::device_vector<double>& likelihoods, int n_gaussians, int n_points) const;
    };

}

#endif
