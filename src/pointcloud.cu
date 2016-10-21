
#include <math.h>
#include <cstdio>
#include <chrono>
#include <limits>
#include <glog/logging.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>

#include <armadillo>

#include "strided_range.h"
#include "point.cuh"
#include "pointcloud.h"
#include "raw_covariance_matrix.h"

using namespace std::chrono;

namespace pclem {

    PointCloud::PointCloud(std::vector<AssociatedPoint> data) :
        data(data), n_points(data.size()) {
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
        VLOG(2) << "Building point cloud...";

        vtkIdType npoints = vtkData->GetNumberOfPoints();
        vtkPoints* points = vtkData->GetPoints();

        auto stl_vec = std::vector<AssociatedPoint>();
        for(int i=0; i < npoints; i++) {
            double currentPoint[3];

            points->GetPoint(i, currentPoint);
            stl_vec.push_back(AssociatedPoint(currentPoint[0], currentPoint[1], currentPoint[2]));
        }

        VLOG(2) << "Done.";
        return PointCloud(stl_vec);
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

    thrust::device_vector<AssociatedPoint>::const_iterator PointCloud::begin() const {
        return data.begin();
    }

    thrust::device_vector<AssociatedPoint>::const_iterator PointCloud::end() const {
        return data.end();
    }

    int PointCloud::get_n_points() const {
        return n_points;
    }

    void PointCloud::compute_associations(const GaussianMixture& mixture) {
        VLOG(10) << "Computing point/distribution associations...";

        for(int i = 0; i < mixture.n_gaussians(); i++) {
            compute_associations_of_distribution(i, mixture.get_gaussian(i));
        }

        VLOG(10) << "Done computing point/distribution associations.";
    }

    struct gaussian_op : public thrust::unary_function<AssociatedPoint,AssociatedPoint> {
    public:
        gaussian_op(int index_of_distribution, const WeightedGaussian& distribution) :
            index_of_distribution(index_of_distribution),
            mu(distribution.get_mu()),
            base(distribution.get_weight() / sqrt(pow(2*M_PI,3) * distribution.get_sigma().det())),
            inv_of_covariance {0.0} {
            std::array<double,9> computed_inv = distribution.get_sigma().inverse();

            for (int i = 0; i < 9; i++) {
                inv_of_covariance[i] = computed_inv[i];
            }
        }

        __host__ __device__
        AssociatedPoint operator()(AssociatedPoint p) {
            p.likelihoods[index_of_distribution] = likelihood_of_point(p);
            return p;
        }

    private:
        __const__ int index_of_distribution;
        __const__ Point mu;
        __const__ double base;
        double inv_of_covariance[9];

        __host__ __device__
        double likelihood_of_point(Point p) {
            Point x_minus_mu = p - mu;

            double temp_product[3] = {x_minus_mu.z*inv_of_covariance[6] + x_minus_mu.y*inv_of_covariance[3] + x_minus_mu.x*inv_of_covariance[0],
                                      x_minus_mu.z*inv_of_covariance[7] + x_minus_mu.y*inv_of_covariance[4] + x_minus_mu.x*inv_of_covariance[1],
                                      x_minus_mu.z*inv_of_covariance[8] + x_minus_mu.y*inv_of_covariance[5] + x_minus_mu.x*inv_of_covariance[2]};

            double scale_product = x_minus_mu.x * temp_product[0] +
                x_minus_mu.y * temp_product[1] +
                x_minus_mu.z * temp_product[2];

            return base * exp(-0.5 * scale_product);
        }
    };

    void PointCloud::compute_associations_of_distribution(int index_of_distribution, const WeightedGaussian& distribution) {
        gaussian_op op(index_of_distribution, distribution);

        thrust::transform(data.begin(), data.end(), data.begin(), op);
    }

    struct normalization_op : public thrust::unary_function<AssociatedPoint, AssociatedPoint> {
        __host__ __device__
        AssociatedPoint operator()(AssociatedPoint p) {
            double sum_of_associations = 0.0;
            for(int i = 0; i < p.N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                sum_of_associations += p.likelihoods[i];
            }

            for(int i = 0; i < p.N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                p.likelihoods[i] = p.likelihoods[i] / sum_of_associations;
            }

            return p;
        }
    };

    void PointCloud::normalize_associations() {
        VLOG(10) << "Normalizing associations for every point...";

        thrust::transform(data.begin(), data.end(), data.begin(), normalization_op());

        VLOG(10) << "Done normalizing associations";
    }

    // This functor stores the sums of gammas in an AssociatedPoint.
    struct sums_of_gammas_op : public thrust::binary_function<AssociatedPoint, AssociatedPoint, AssociatedPoint> {

        __host__ __device__
        AssociatedPoint operator()(AssociatedPoint lhs, AssociatedPoint rhs) {
            AssociatedPoint sum;
            for(int i=0; i < 8; i++) {
                sum.likelihoods[i] = lhs.likelihoods[i] + rhs.likelihoods[i];
            }
            return sum;
        }
    };

    GaussianMixture PointCloud::create_mixture() const {
        // We store the sum of gammas of every distribution in an empty, meaningless AssociatedPoint.
        AssociatedPoint sums;
        sums = thrust::reduce(data.begin(), data.end(), AssociatedPoint(), sums_of_gammas_op());

        std::vector<WeightedGaussian> gaussians;
        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            gaussians.push_back(create_distribution_of_mixture(i, sums.likelihoods[i]));
        }

        return GaussianMixture(gaussians);
    }

    struct weight_point_op : public thrust::unary_function<AssociatedPoint,Point> {
        int index_of_distribution;

        weight_point_op(int index_of_distribution) : index_of_distribution(index_of_distribution) {}

        __host__ __device__
        Point operator()(AssociatedPoint p) {
            return Point(p.x * p.likelihoods[index_of_distribution],
                         p.y * p.likelihoods[index_of_distribution],
                         p.z * p.likelihoods[index_of_distribution]);
        }
    };

    struct sum_of_points_op : public thrust::binary_function<Point,Point,Point> {
        __host__ __device__
        Point operator()(Point lhs, Point rhs) {
            return lhs + rhs;
        }
    };

    WeightedGaussian PointCloud::create_distribution_of_mixture(int index_of_distribution, double sum_of_gammas) const {
        VLOG(10) << "Creating distribution " << index_of_distribution << " of mixture...";

        Point new_mu = thrust::transform_reduce(data.begin(), data.end(),
                                                weight_point_op(index_of_distribution),
                                                Point(0.0, 0.0, 0.0),
                                                sum_of_points_op());

        new_mu = Point(new_mu.x / sum_of_gammas,
                       new_mu.y / sum_of_gammas,
                       new_mu.z / sum_of_gammas);

        VLOG(1) << "New mu: " << new_mu;

        CovarianceMatrix new_sigma = compute_sigma(index_of_distribution, new_mu, sum_of_gammas);

        double new_weight = sum_of_gammas / get_n_points();

        VLOG(10) << "Done creating distribution " << index_of_distribution << " of mixture.";
        return WeightedGaussian(new_mu, new_sigma, new_weight);
    }

    struct point_to_cov_op : public thrust::unary_function<AssociatedPoint, RawCovarianceMatrix> {
        int index_of_distribution;

        point_to_cov_op(int index_of_distribution) : index_of_distribution(index_of_distribution) {}

        __host__ __device__
        RawCovarianceMatrix operator()(AssociatedPoint p) {
            RawCovarianceMatrix r;
            double gamma = p.likelihoods[index_of_distribution];

            r.v00 = p.x * p.x * gamma;
            r.v11 = p.y * p.y * gamma;
            r.v22 = p.z * p.z * gamma;

            r.v10 = r.v01 = p.x * p.y * gamma;
            r.v20 = r.v02 = p.x * p.z * gamma;
            r.v21 = r.v12 = p.y * p.z * gamma;

            return r;
        }
    };

    struct sum_of_cov_op : public thrust::binary_function<RawCovarianceMatrix, RawCovarianceMatrix, RawCovarianceMatrix> {
        __host__ __device__
        RawCovarianceMatrix operator()(RawCovarianceMatrix lhs, RawCovarianceMatrix rhs) {
            return lhs + rhs;
        }
    };

    CovarianceMatrix PointCloud::compute_sigma(int index_of_distribution, const Point& mu, double sum_of_gammas) const {
        RawCovarianceMatrix init = RawCovarianceMatrix::zeros();

        RawCovarianceMatrix sigma = thrust::transform_reduce(data.begin(), data.end(),
                                                             point_to_cov_op(index_of_distribution),
                                                             init, sum_of_cov_op());

        RawCovarianceMatrix base_sigma;

        base_sigma.v00 = mu.x * mu.x;
        base_sigma.v11 = mu.y * mu.y;
        base_sigma.v22 = mu.z * mu.z;

        base_sigma.v01 = base_sigma.v10 = mu.x * mu.y;
        base_sigma.v02 = base_sigma.v20 = mu.x * mu.z;
        base_sigma.v21 = base_sigma.v12 = mu.z * mu.y;

        sigma = sigma / sum_of_gammas - base_sigma;
        return CovarianceMatrix(sigma);
    }

    double PointCloud::log_likelihood_of_mixture(const GaussianMixture& mixture) const {
        double log_likelihood = 0.0;

        std::cout << "Log likelihood of distributions: ";
        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            double likelihood_of_distribution = log_likelihood_of_distribution(i, mixture.get_gaussian(i));

            std::cout << likelihood_of_distribution << " ";

            log_likelihood += likelihood_of_distribution;
        }
        std::cout << std::endl;

        return log_likelihood;
    }

    struct log_likelihood_op : public thrust::unary_function<AssociatedPoint,double> {
        __const__ double log_pi_ij;
        __const__ double base;
        __const__ Point mu;
        __const__ int index_of_distribution;
        double inv_of_cov[9];

        log_likelihood_op(int index_of_distribution, const WeightedGaussian& distribution) :
            log_pi_ij(distribution.get_weight()),
            base(1.0 / sqrt(pow(2*M_PI, 3) * distribution.get_sigma().det())),
            mu(distribution.get_mu()),
            index_of_distribution(index_of_distribution),
            inv_of_cov {0.0} {
            std::array<double,9> inv_of_sigma = distribution.get_sigma().inverse();

            for(int i=0; i < 9; i++) {
                inv_of_cov[i] = inv_of_sigma[i];
            }
        }

        __host__ __device__
        double operator()(AssociatedPoint p) {
            if(p.likelihoods[index_of_distribution] < 1e-70) {
                return 0.0;
            } else {
                return p.likelihoods[index_of_distribution] * (log_pi_ij + log(likelihood_of_point(p)));
            }
        }

    private:
        __host__ __device__
        double likelihood_of_point(Point p) {
            Point x_minus_mu = p - mu;

            double temp_product[3] = {x_minus_mu.z*inv_of_cov[6] + x_minus_mu.y*inv_of_cov[3] + x_minus_mu.x*inv_of_cov[0],
                                      x_minus_mu.z*inv_of_cov[7] + x_minus_mu.y*inv_of_cov[4] + x_minus_mu.x*inv_of_cov[1],
                                      x_minus_mu.z*inv_of_cov[8] + x_minus_mu.y*inv_of_cov[5] + x_minus_mu.x*inv_of_cov[2]};

            double scale_product = x_minus_mu.x * temp_product[0] +
                x_minus_mu.y * temp_product[1] +
                x_minus_mu.z * temp_product[2];

            return base * exp(-0.5 * scale_product);
        }
    };

    double PointCloud::log_likelihood_of_distribution(int index_of_distribution, const WeightedGaussian& distribution) const  {
        return thrust::transform_reduce(data.begin(), data.end(),
                                        log_likelihood_op(index_of_distribution, distribution),
                                        0.0, thrust::plus<double>());
    }
}
