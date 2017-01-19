
#include <glog/logging.h>
#include <thrust/transform_reduce.h>

#include "associated_point.cuh"
#include "log_likelihood_operation.h"

namespace pclem {
    LogLikelihoodOperation::LogLikelihoodOperation(const GaussianMixture& mixture) :
        mixture(mixture) {}

    double LogLikelihoodOperation::operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) const {
        double log_likelihood = 0.0;

        for(int i=0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            WeightedGaussian distribution = mixture.get_gaussian(i);

            double likelihood_of_distribution = 0.0;
            if(distribution.get_weight() != 0.0) {
                likelihood_of_distribution = log_likelihood_of_distribution(begin, end,
                                                                            i, distribution);
            }

            VLOG(4) << "Mixture was: " << mixture;
            VLOG(4) << "Likelihood was: " << likelihood_of_distribution;

            log_likelihood += likelihood_of_distribution;
        }

        return log_likelihood;
    }

    struct log_likelihood_op : public thrust::unary_function<AssociatedPoint,double> {
        __const__ double log_pi_j;
        __const__ double base;
        __const__ DevicePoint mu;
        __const__ int index_of_distribution;
        double inv_of_cov[9];

        log_likelihood_op(int index_of_distribution, const WeightedGaussian& distribution) :
            log_pi_j(distribution.get_weight()),
            base(1.0 / sqrt(pow(2*M_PI, 3) * distribution.get_sigma().det())),
            mu(distribution.get_mu()),
            index_of_distribution(index_of_distribution),
            inv_of_cov {0.0} {

            VLOG(12) << "To inverse: " << distribution.get_sigma();
            std::array<double,9> inv_of_sigma = distribution.get_sigma().inverse();

            for(int i=0; i < 9; i++) {
                inv_of_cov[i] = inv_of_sigma[i];
            }
        }

        __host__ __device__
        double operator()(AssociatedPoint p) {
            if(p.associations[index_of_distribution] < 1e-30) {
                return 0.0;
            } else {
                return p.associations[index_of_distribution] * (log_pi_j + log(likelihood_of_point(p)));
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

    double LogLikelihoodOperation::log_likelihood_of_distribution(const DevicePointCloud::PointIterator& begin,
                                                                  const DevicePointCloud::PointIterator& end,
                                                                  int index_of_distribution, const WeightedGaussian& distribution) const {
        return thrust::transform_reduce(begin, end,
                                        log_likelihood_op(index_of_distribution, distribution),
                                        0.0, thrust::plus<double>());
    }
}
