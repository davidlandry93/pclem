
#include <glog/logging.h>

#include <thrust/transform.h>

#include "association_computing_operation.h"

namespace pclem {
    
    AssociationComputingOperation::AssociationComputingOperation(const GaussianMixture& mixture, const double& volume) :
        mixture(mixture), volume_of_pcl(volume) {}

    void AssociationComputingOperation::operator()(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end) {
        VLOG(10) << "Computing point/distribution associations...";

        for(int i = 0; i < AssociatedPoint::N_DISTRIBUTIONS_PER_MIXTURE; i++) {
            select_and_execute_op(begin, end, i);
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
            p.associations[index_of_distribution] = likelihood_of_point(p);

            if(p.associations[index_of_distribution] > p.associations[p.best_distribution]) {
                p.best_distribution = index_of_distribution;
            }

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

    struct fixed_association_op : public thrust::unary_function<AssociatedPoint,AssociatedPoint> {
        int index_of_distribution;
        double association_value;

        fixed_association_op(int index_of_distribution, double association_value) :
            index_of_distribution(index_of_distribution),
            association_value(association_value) {}

        __host__ __device__
        AssociatedPoint operator()(AssociatedPoint p) {
            p.associations[index_of_distribution] = association_value;
            return p;
        }
    };

    void AssociationComputingOperation::select_and_execute_op(const DevicePointCloud::PointIterator& begin, const DevicePointCloud::PointIterator& end, const int& i) {
        if(mixture.get_gaussian(i).get_weight() == 0.0) {
            thrust::transform(begin, end, begin, fixed_association_op(i, 0.0));
        } else {
            thrust::transform(begin, end, begin, gaussian_op(i, mixture.get_gaussian(i)));
        }
    }
}
