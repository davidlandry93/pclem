
#include <math.h>
#include <cstdio>
#include <chrono>
#include <limits>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>

#include <armadillo>

#include "strided_range.h"
#include "pointcloud.h"
#include "point.cuh"

using namespace std::chrono;

namespace pclem {

    PointCloud::PointCloud(std::vector<Point> data, int npoints) :
        data(data), n_points(npoints) {
        updateBoundingBox();
    }

    PointCloud::PointCloud(PointCloud&& other) :
        data(), n_points(other.n_points), boundingBox(other.boundingBox) {
        std::swap(data,other.data);
    }

    PointCloud& PointCloud::operator=(PointCloud&& other) {
        std::swap(data, other.data);
        return *this;
    }

    PointCloud PointCloud::from_vtk(vtkPolyData* vtkData) {
        vtkIdType npoints = vtkData->GetNumberOfPoints();
        vtkPoints* points = vtkData->GetPoints();

        auto stl_vec = std::vector<Point>();
        for(int i=0; i < npoints; i++) {
            double currentPoint[3];

            points->GetPoint(i, currentPoint);
            stl_vec.push_back(Point(currentPoint[0], currentPoint[1], currentPoint[2]));
        }

        return PointCloud(stl_vec, npoints);
    }

    BoundingBox PointCloud::getBoundingBox() {
        return boundingBox;
    }

    struct min_op: public thrust::binary_function<Point,Point,Point> {
        __device__
        Point operator()(Point lhs, Point rhs) {
            return Point(thrust::min(lhs.x, rhs.x),
                         thrust::min(lhs.y, rhs.y),
                         thrust::min(lhs.z, rhs.z));
        }
    };

    struct max_op: public thrust::binary_function<Point,Point,Point> {
        __device__
        Point operator()(Point lhs, Point rhs) {
            return Point(thrust::max(lhs.x, rhs.x),
                         thrust::max(lhs.y, rhs.y),
                         thrust::max(lhs.z, rhs.z));
        }
    };

    void PointCloud::updateBoundingBox(){
        min_op min_function;
        max_op max_function;

        Point min = thrust::reduce(data.begin(), data.end(), Point(0.0,0.0,0.0), min_function);
        Point max = thrust::reduce(data.begin(), data.end(), Point(0.0,0.0,0.0), max_function);

        boundingBox.setMin(min);
        boundingBox.setMax(max);
    }


    struct normalization_op : public thrust::unary_function<double,double> {
        double operator()(double likelihood) {
            return 0.0;
        }
    };

    thrust::device_vector<Point>::const_iterator PointCloud::begin() const {
        return data.begin();
    }

    thrust::device_vector<Point>::const_iterator PointCloud::end() const {
        return data.end();
    }

    void PointCloud::normalize_likelihoods(thrust::device_vector<double>& likelihoods,
                                           int n_gaussians,
                                           int n_points) const {
        typedef thrust::device_vector<double>::iterator Iterator;

        for(int i = 0; i < n_points; i++) {
            StridedRange<Iterator> range(likelihoods.begin() + i, likelihoods.end(), n_points);

            double sum = thrust::reduce(range.begin(), range.end(), 0.0, thrust::plus<double>());
            std::cout << sum << std::endl;
        }
    }

    int PointCloud::get_n_points() const {
        return n_points;
    }
}
