#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>
#include <vtkPolyData.h>

#include <thrust/device_vector.h>

#include "point.cuh"
#include "boundingbox.h"
#include "gaussian_mixture.h"

namespace pclem {

    class PointCloud {
    public:
        static PointCloud from_vtk(vtkPolyData* vtkData);
        PointCloud(PointCloud&& other);
        PointCloud& operator=(PointCloud&& other);
        BoundingBox getBoundingBox();
        void likelihoods(const GaussianMixture& mixture,
                         thrust::device_vector<double>& result) const;
        void likelihoods_of_distribution(WeightedGaussian gaussian,
                                         thrust::device_vector<double>::iterator result) const;
        int get_n_points() const;
    private:
        thrust::device_vector<Point> data;
        int n_points;
        BoundingBox boundingBox;

        PointCloud(std::vector<Point> data, int n_of_points);
        PointCloud(PointCloud& other);
        void updateBoundingBox();
    };

}

#endif
