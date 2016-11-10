
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
#include "device_pointcloud.h"
#include "raw_covariance_matrix.cuh"
#include "gaussian_mixture_factory.h"
#include "device_hierarchical_gaussian_mixture.h"

namespace pclem {
    DevicePointCloud::DevicePointCloud() :
        ptr_to_points(new thrust::device_vector<AssociatedPoint>()),
        pts_begin(ptr_to_points->begin()),
        pts_end(ptr_to_points->end()),
        boundingBox() {}

    DevicePointCloud::DevicePointCloud(const DevicePointCloud& other) :
        ptr_to_points(other.ptr_to_points),
        pts_begin(other.pts_begin),
        pts_end(other.pts_end),
        boundingBox(other.boundingBox){}

    BoundingBox DevicePointCloud::getBoundingBox() const {
        return BoundingBox(boundingBox);
    }

    struct min_op: public thrust::binary_function<DevicePoint,DevicePoint,DevicePoint> {
        __device__
        DevicePoint operator()(DevicePoint lhs, DevicePoint rhs) {
            return DevicePoint(thrust::min(lhs.x, rhs.x),
                               thrust::min(lhs.y, rhs.y),
                               thrust::min(lhs.z, rhs.z));
        }
    };

    struct max_op: public thrust::binary_function<DevicePoint,DevicePoint,DevicePoint> {
        __device__
        DevicePoint operator()(DevicePoint lhs, DevicePoint rhs) {
            return DevicePoint(thrust::max(lhs.x, rhs.x),
                               thrust::max(lhs.y, rhs.y),
                               thrust::max(lhs.z, rhs.z));
        }
    };

    void DevicePointCloud::updateBoundingBox(){
        min_op min_function;
        max_op max_function;

        DevicePoint min = thrust::reduce(pts_begin, pts_end, DevicePoint(0.0,0.0,0.0), min_function);
        DevicePoint max = thrust::reduce(pts_begin, pts_end, DevicePoint(0.0,0.0,0.0), max_function);

        Point host_min = min.to_host();
        Point host_max = max.to_host();

        boundingBox.setMin(host_min);
        boundingBox.setMax(host_max);
    }

    int DevicePointCloud::get_n_points() const {
        return pts_end - pts_begin;
    }

    void DevicePointCloud::set_points(const std::shared_ptr<thrust::device_vector<AssociatedPoint>>& points) {
        VLOG(10) << "Setting new data source...";

        ptr_to_points = points;
        pts_begin = points->begin();
        pts_end = points->end();

        updateBoundingBox();

        VLOG(10) << "Done setting data source.";
    }

    void DevicePointCloud::set_points(const std::shared_ptr<thrust::device_vector<AssociatedPoint>>& points,
                                      const PointIterator& begin,
                                      const PointIterator& end) {
        ptr_to_points = points;
        pts_begin = begin;
        pts_end = end;
    }

    void DevicePointCloud::compute_associations(const GaussianMixture& mixture) {
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
        __const__ DevicePoint mu;
        __const__ double base;
        double inv_of_covariance[9];

        __host__ __device__
        double likelihood_of_point(DevicePoint p) {
            DevicePoint x_minus_mu = p - mu;

            double temp_product[3] = {x_minus_mu.z*inv_of_covariance[6] + x_minus_mu.y*inv_of_covariance[3] + x_minus_mu.x*inv_of_covariance[0],
                                      x_minus_mu.z*inv_of_covariance[7] + x_minus_mu.y*inv_of_covariance[4] + x_minus_mu.x*inv_of_covariance[1],
                                      x_minus_mu.z*inv_of_covariance[8] + x_minus_mu.y*inv_of_covariance[5] + x_minus_mu.x*inv_of_covariance[2]};

            double scale_product = x_minus_mu.x * temp_product[0] +
                x_minus_mu.y * temp_product[1] +
                x_minus_mu.z * temp_product[2];

            return base * exp(-0.5 * scale_product);
        }
    };

    void DevicePointCloud::compute_associations_of_distribution(int index_of_distribution, const WeightedGaussian& distribution) {
        gaussian_op op(index_of_distribution, distribution);

        thrust::transform(pts_begin, pts_end, pts_begin, op);
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

    void DevicePointCloud::normalize_associations() {
        VLOG(10) << "Normalizing associations for every point...";

        thrust::transform(pts_begin, pts_end, pts_begin, normalization_op());

        VLOG(10) << "Done normalizing associations";
    }

    // This functor stores the sums of gammas in an AssociatedPoint.
    struct sums_of_gammas_op : public thrust::binary_function<AssociatedPoint, AssociatedPoint, AssociatedPoint> {

        __host__ __device__
        AssociatedPoint operator()(AssociatedPoint lhs, AssociatedPoint rhs) {
            AssociatedPoint sum;
            for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                sum.likelihoods[i] = lhs.likelihoods[i] + rhs.likelihoods[i];
            }
            return sum;
        }
    };

    GaussianMixture DevicePointCloud::create_mixture() const {
        // We store the sum of gammas of every distribution in an empty, meaningless AssociatedPoint.
        AssociatedPoint sums;
        sums = thrust::reduce(pts_begin, pts_end, AssociatedPoint(), sums_of_gammas_op());

        std::vector<WeightedGaussian> gaussians;

        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            gaussians.push_back(create_distribution_of_mixture(i, sums.likelihoods[i]));
        }

        return GaussianMixture(gaussians);
    }

    struct weight_point_op : public thrust::unary_function<AssociatedPoint,DevicePoint> {
        int index_of_distribution;

        weight_point_op(int index_of_distribution) : index_of_distribution(index_of_distribution) {}

        __host__ __device__
        DevicePoint operator()(AssociatedPoint p) {
            return DevicePoint(p.x * p.likelihoods[index_of_distribution],
                               p.y * p.likelihoods[index_of_distribution],
                               p.z * p.likelihoods[index_of_distribution]);
        }
    };

    struct sum_of_points_op : public thrust::binary_function<DevicePoint,DevicePoint,DevicePoint> {
        __host__ __device__
        DevicePoint operator()(DevicePoint lhs, DevicePoint rhs) {
            return lhs + rhs;
        }
    };

    WeightedGaussian DevicePointCloud::create_distribution_of_mixture(int index_of_distribution, double sum_of_gammas) const {
        VLOG(10) << "Creating distribution " << index_of_distribution << " of mixture...";

        DevicePoint new_mu = thrust::transform_reduce(pts_begin, pts_end,
                                                      weight_point_op(index_of_distribution),
                                                      DevicePoint(0.0, 0.0, 0.0),
                                                      sum_of_points_op());

        new_mu = DevicePoint(new_mu.x / sum_of_gammas,
                             new_mu.y / sum_of_gammas,
                             new_mu.z / sum_of_gammas);

        VLOG(1) << "New mu: " << new_mu;

        CovarianceMatrix new_sigma = compute_sigma(index_of_distribution, new_mu.to_host(), sum_of_gammas);

        double new_weight = sum_of_gammas / get_n_points();

        VLOG(10) << "Done creating distribution " << index_of_distribution << " of mixture.";

        Point pub_new_mu = new_mu.to_host();
        WeightedGaussian to_return = WeightedGaussian(pub_new_mu, new_sigma, new_weight);

        return to_return;
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

    CovarianceMatrix DevicePointCloud::compute_sigma(int index_of_distribution, const Point& mu, double sum_of_gammas) const {
        RawCovarianceMatrix init = RawCovarianceMatrix::zeros();

        RawCovarianceMatrix sigma = thrust::transform_reduce(pts_begin, pts_end,
                                                             point_to_cov_op(index_of_distribution),
                                                             init, sum_of_cov_op());

        RawCovarianceMatrix base_sigma;

        base_sigma.v00 = mu.x * mu.x;
        base_sigma.v11 = mu.y * mu.y;
        base_sigma.v22 = mu.z * mu.z;

        base_sigma.v01 = base_sigma.v10 = mu.x * mu.y;
        base_sigma.v02 = base_sigma.v20 = mu.x * mu.z;
        base_sigma.v21 = base_sigma.v12 = mu.z * mu.y;

        sigma = (sigma / sum_of_gammas) - base_sigma;
        return sigma.to_host();
    }

    double DevicePointCloud::log_likelihood_of_mixture(const GaussianMixture& mixture) const {
        double log_likelihood = 0.0;

        // std::cout << "Log likelihood of distributions: ";
        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            double likelihood_of_distribution = log_likelihood_of_distribution(i, mixture.get_gaussian(i));

            // std::cout << likelihood_of_distribution << " ";

            log_likelihood += likelihood_of_distribution;
        }
        // std::cout << std::endl;

        return log_likelihood;
    }

    struct log_likelihood_op : public thrust::unary_function<AssociatedPoint,double> {
        __const__ double log_pi_ij;
        __const__ double base;
        __const__ DevicePoint mu;
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
            if(p.likelihoods[index_of_distribution] < 1e-30) {
                return 0.0;
            } else {
                return p.likelihoods[index_of_distribution] * (log_pi_ij + log(likelihood_of_point(p)));
            }
        }

    private:
        __host__ __device__
        double likelihood_of_point(DevicePoint p) {
            DevicePoint x_minus_mu = p - mu;

            double temp_product[3] = {x_minus_mu.z*inv_of_cov[6] + x_minus_mu.y*inv_of_cov[3] + x_minus_mu.x*inv_of_cov[0],
                                      x_minus_mu.z*inv_of_cov[7] + x_minus_mu.y*inv_of_cov[4] + x_minus_mu.x*inv_of_cov[1],
                                      x_minus_mu.z*inv_of_cov[8] + x_minus_mu.y*inv_of_cov[5] + x_minus_mu.x*inv_of_cov[2]};

            double scale_product = x_minus_mu.x * temp_product[0] +
                x_minus_mu.y * temp_product[1] +
                x_minus_mu.z * temp_product[2];

            return base * exp(-0.5 * scale_product);
        }
    };

    double DevicePointCloud::log_likelihood_of_distribution(int index_of_distribution, const WeightedGaussian& distribution) const  {
        return thrust::transform_reduce(pts_begin, pts_end,
                                        log_likelihood_op(index_of_distribution, distribution),
                                        0.0, thrust::plus<double>());
    }

    std::vector<Point> DevicePointCloud::copy_of_points() const {
        std::vector<Point> copy;

        for(auto it = pts_begin; it != pts_end; it++) {
            AssociatedPoint device_point(*it);
            copy.push_back(device_point.to_host());
        }

        return copy;
    }

    HierarchicalGaussianMixture DevicePointCloud::create_hgmm() {
        VLOG(10) << "Creating hgmm...";
        GaussianMixtureFactory gmm_factory;

        std::shared_ptr<DeviceHierarchicalGaussianMixture> hierarchical_mixture(new DeviceHierarchicalGaussianMixture(*this, gmm_factory.from_pcl_corners(*this)));
        hierarchical_mixture->create_children();

        VLOG(10) << "Done creating hgmm.";
        return HierarchicalGaussianMixture(hierarchical_mixture);
    }

    DevicePointCloud::PointIterator DevicePointCloud::begin() {
        return pts_begin;
    }

    DevicePointCloud::PointIterator DevicePointCloud::end() {
        return pts_end;
    }
}
