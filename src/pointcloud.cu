
#include <cstdio>
#include <chrono>
#include <limits>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>

#include "pointcloud.h"
#include "point.h"

using namespace std::chrono;

namespace pclem {

    PointCloud::PointCloud(std::vector<Point> data, int npoints) :
        data(data), n_points(npoints) {
        updateBoundingBox();
    }

    PointCloud::PointCloud(PointCloud&& other) :
        n_points(other.n_points), boundingBox(other.boundingBox) {
        data = std::move(data);
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

    void PointCloud::likelihoods(const GaussianMixture& mixture, thrust::device_vector<double>& result) {
        int n_gaussians = mixture.n_gaussians();
        int i = 0;
        for(auto gaussian : mixture) {
            WeightedGaussian host_gaussian = gaussian;
            std::cout << "Sigma in mixture" << host_gaussian.get_sigma().det();

            likelihoods_of_distribution(gaussian, result.begin() + i*n_gaussians);
            i++;
        }
    }

    class gaussian_op : public thrust::unary_function<Point,double> {
    public:
        __host__
        gaussian_op(const WeightedGaussian& _g) : g(_g) {
            det_sigma = g.get_sigma().det();
        }
        __device__
        double operator()(Point p) {
            //printf("%d", det_sigma);
            return det_sigma;
        }

    private:
        WeightedGaussian g;
        double det_sigma;
    };

    void PointCloud::likelihoods_of_distribution(WeightedGaussian gaussian,
                                                 thrust::device_vector<double>::iterator result) {
        gaussian_op op(gaussian);

        //thrust::transform(data.begin(), data.end(), result, op);
    }

    int PointCloud::get_n_points() const {
        return n_points;
    }
}
