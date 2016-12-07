
#include <glog/logging.h>
#include <thrust/transform_reduce.h>

#include "mixture_creation_operation.h"
#include "raw_covariance_matrix.cuh"

namespace pclem {

    MixtureCreationOperation::MixtureCreationOperation(double weight_of_parent_in_hierarchy) :
        weight_of_parent_in_hierarchy(weight_of_parent_in_hierarchy) {}
    // This functor stores the sums of gammas in an AssociatedPoint.
    struct sums_of_gammas_op : public thrust::binary_function<AssociatedPoint, AssociatedPoint, AssociatedPoint> {

        __host__ __device__
        AssociatedPoint operator()(AssociatedPoint lhs, AssociatedPoint rhs) {
            AssociatedPoint sum;
            for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
                sum.associations[i] = lhs.associations[i] + rhs.associations[i];
            }
            return sum;
        }
    };

    GaussianMixture MixtureCreationOperation::operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) const {

        // We store the sum of gammas of every distribution in an empty, meaningless AssociatedPoint.
        AssociatedPoint sums;
        sums = thrust::reduce(begin, end, AssociatedPoint(0.0,0.0,0.0), sums_of_gammas_op());

        std::vector<WeightedGaussian> gaussians;

        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            gaussians.push_back(create_distribution_of_mixture(begin, end, i, sums.associations[i]));
        }

        GaussianMixture mixture(gaussians);

        return mixture;
    }

    struct weight_point_op : public thrust::unary_function<AssociatedPoint,DevicePoint> {
        int index_of_distribution;

        weight_point_op(int index_of_distribution) : index_of_distribution(index_of_distribution) {}

        __host__ __device__
        DevicePoint operator()(AssociatedPoint p) {
            return DevicePoint(p.x * p.associations[index_of_distribution],
                               p.y * p.associations[index_of_distribution],
                               p.z * p.associations[index_of_distribution]);
        }
    };

    struct sum_of_points_op : public thrust::binary_function<DevicePoint,DevicePoint,DevicePoint> {
        __host__ __device__
        DevicePoint operator()(DevicePoint lhs, DevicePoint rhs) {
            return lhs + rhs;
        }
    };

    WeightedGaussian MixtureCreationOperation::create_distribution_of_mixture(const DevicePointCloud::PointIterator& begin,
                                                                              const DevicePointCloud::PointIterator& end,
                                                                              int index_of_distribution, double sum_of_gammas) const {
        VLOG(11) << "Creating distribution " << index_of_distribution << " of mixture...";

        DevicePoint new_mu = thrust::transform_reduce(begin, end,
                                                      weight_point_op(index_of_distribution),
                                                      DevicePoint(0.0, 0.0, 0.0),
                                                      sum_of_points_op());

        new_mu = DevicePoint(new_mu.x / sum_of_gammas,
                             new_mu.y / sum_of_gammas,
                             new_mu.z / sum_of_gammas);

        VLOG(11) << "New mu: " << new_mu;

        CovarianceMatrix new_sigma = compute_sigma(begin, end, index_of_distribution, new_mu.to_host(), sum_of_gammas);

        VLOG(7) << "Sum of gammas: " << sum_of_gammas;
        double new_weight = 0.0;
        if(sum_of_gammas > DROPOUT_WEIGHT) {
            new_weight = sum_of_gammas / (end - begin);
        } else {
            VLOG(1) << "Dropping distribution";
        }

        VLOG(11) << "Done creating distribution " << index_of_distribution << " of mixture.";

        Point pub_new_mu = new_mu.to_host();
        WeightedGaussian to_return = WeightedGaussian(pub_new_mu, new_sigma, new_weight, weight_of_parent_in_hierarchy);

        return to_return;
    }

    struct point_to_cov_op : public thrust::unary_function<AssociatedPoint, RawCovarianceMatrix> {
        int index_of_distribution;

        point_to_cov_op(int index_of_distribution) : index_of_distribution(index_of_distribution) {}

        __host__ __device__
        RawCovarianceMatrix operator()(AssociatedPoint p) {
            RawCovarianceMatrix r;
            double gamma = p.associations[index_of_distribution];

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

    CovarianceMatrix MixtureCreationOperation::compute_sigma(const DevicePointCloud::PointIterator& begin,
                                                             const DevicePointCloud::PointIterator& end,
                                                             int index_of_distribution, const Point& mu, double sum_of_gammas) const {
        RawCovarianceMatrix init = RawCovarianceMatrix::zeros();

        RawCovarianceMatrix sigma = thrust::transform_reduce(begin, end,
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
}
