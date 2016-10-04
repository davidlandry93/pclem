
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

    void PointCloud::likelihoods(const GaussianMixture& mixture, thrust::device_vector<double>& result) const {
        int n_gaussians = mixture.n_gaussians();

        for(int i=0; i < n_gaussians; i++) {
            WeightedGaussian g = mixture.get_gaussian(i);

            likelihoods_of_distribution(mixture.get_gaussian(i), result.begin() + i*n_points);
            normalize_likelihoods(result, n_gaussians, n_points);
        }
    }

    struct gaussian_op : public thrust::unary_function<Point,double> {
    public:
        gaussian_op(Point _mu, double _det, std::array<double,9> _inv_of_covariance) :
            mu(_mu),
            base(1 / sqrt(pow(2*M_PI, 3) * _det)),
            inv() {
            for(int i = 0; i < 9; i++) {
                inv[i] = _inv_of_covariance[i];
            }


            std::cout << _mu;
            std::cout << _det;
        }


        // https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Density_function
        __host__ __device__
        double operator()(Point x) {
            Point x_minus_mu = x - mu;
            double temp_product[3] = {x_minus_mu.z*inv[6] + x_minus_mu.y*inv[3] + x_minus_mu.x*inv[0],
                                      x_minus_mu.z*inv[7] + x_minus_mu.y*inv[4] + x_minus_mu.x*inv[1],
                                      x_minus_mu.z*inv[8] + x_minus_mu.y*inv[5] + x_minus_mu.x*inv[2]};

            double scale_product =
                x_minus_mu.x*temp_product[0] +
                x_minus_mu.y*temp_product[1] +
                x_minus_mu.z*temp_product[2];

            return base * exp(-0.5 * scale_product);
        }

        __const__ Point mu;
        __const__ double base;
        double inv[9];
    };

    void PointCloud::likelihoods_of_distribution(WeightedGaussian gaussian,
                                                 thrust::device_vector<double>::iterator result) const {
        std::array<double,9> cov_mat = gaussian.get_sigma().as_array();

        arma::mat33 arma_cov_mat(cov_mat.data());
        double det_of_covariance = arma::det(arma_cov_mat);

        arma::mat33 arma_inv_of_covariance = arma::inv_sympd(arma_cov_mat);

        std::array<double,9> inv_of_covariance;
        for(auto i = 0; i < 3; i++) {
            for (auto j=0; j < 3; j++) {
                inv_of_covariance[i*3+j] = arma_inv_of_covariance(i,j);
            }
        }

        gaussian_op op(gaussian.get_mu(), det_of_covariance, inv_of_covariance);

        std::cout << "callin" << std::endl;
        thrust::transform(data.begin(), data.end(), result, op);

        std::cout.precision(17);
        std::cout << std::scientific;
        std::cout << *result << " " << *(result + 1) << std::endl;
        std::cout << "Max: " << *(thrust::max_element(result, result + (data.end() - data.begin()))) << std::endl;
    }

    struct normalization_op : public thrust::unary_function<double,double> {
        double operator()(double likelihood) {
            return 0.0;
        }
    };

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
