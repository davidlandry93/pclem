
#include <glog/logging.h>

#include "device_pointcloud.h"
#include "covariance_matrix.cuh"
#include "em_algorithm.h"

namespace pclem {
    EmAlgorithm::EmAlgorithm(DevicePointCloud& pcl,
                             PrivateGaussianMixture& mixture) :
        pcl(std::move(pcl)),
        mixture(std::move(mixture)) {}

    EmAlgorithm::EmAlgorithm(EmAlgorithm&& other) :
        pcl(std::move(other.pcl)), mixture(std::move(other.mixture)) {}

    EmAlgorithm EmAlgorithm::from_pcl(DevicePointCloud& pcl) {
        std::vector<WeightedGaussian> temp_gaussians;

        auto corners = pcl.getBoundingBox().corners();
        double initial_weight_of_gaussian = 1.0 / corners.size();

        for(Point corner : corners) {
            CovarianceMatrix sigma = CovarianceMatrix();
            sigma.set(0,0,10.0);
            sigma.set(1,1,10.0);
            sigma.set(2,2,10.0);

            WeightedGaussian gaussian(corner, sigma, initial_weight_of_gaussian);
            temp_gaussians.push_back(gaussian);
        }

        PrivateGaussianMixture mixture(temp_gaussians);

        EmAlgorithm temp_em(pcl, mixture);

        return temp_em;
    }

    EmAlgorithm& EmAlgorithm::operator=(EmAlgorithm&& other) {
        std::swap(pcl, other.pcl);
        std::swap(mixture, other.mixture);
        return *this;
    }

    void EmAlgorithm::expectation() {
        VLOG(10) << "Computing expectation...";

        pcl.compute_associations(mixture);
        pcl.normalize_associations();

        VLOG(10) << "Done.";
    }

    void EmAlgorithm::maximization() {
        VLOG(10) << "Computing maximization...";
        mixture = pcl.create_mixture();
        VLOG(10) << "Done.";
    }

    std::ostream& operator<<(std::ostream& os, const EmAlgorithm& em) {
        os << "===EM===" << std::endl << em.mixture;
        return os;
    }

    double EmAlgorithm::log_likelihood() {
        return pcl.log_likelihood_of_mixture(mixture);
    }
}
