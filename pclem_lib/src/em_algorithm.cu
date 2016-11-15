
#include <glog/logging.h>

#include "device_pointcloud.h"
#include "covariance_matrix.h"
#include "em_algorithm.h"
#include "gaussian_mixture_factory.h"

namespace pclem {
    EmAlgorithm::EmAlgorithm(PointCloud& pcl,
                             GaussianMixture& mixture) :
        pcl(std::move(pcl)),
        mixture(std::move(mixture)) {}

    EmAlgorithm::EmAlgorithm(EmAlgorithm&& other) :
        pcl(std::move(other.pcl)), mixture(std::move(other.mixture)) {}

    EmAlgorithm EmAlgorithm::from_pcl(PointCloud& pcl) {
        GaussianMixtureFactory gm_factory;
        GaussianMixture mixture = gm_factory.from_pcl_corners(pcl);

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

    void EmAlgorithm::run(double epsilon) {
        VLOG(10) << "Running expectation maximization...";

        double previous_likelihood = 0.0;
        double delta = std::numeric_limits<double>::infinity();

        while(delta > epsilon ||
              previous_likelihood == -1*std::numeric_limits<double>::infinity()) {
            expectation();
            maximization();

            double new_likelihood = log_likelihood();
            delta = std::abs(new_likelihood - previous_likelihood);
            LOG(INFO) << "Log likelihood: " << new_likelihood;
            LOG(INFO) << "Delta: " << delta;

            previous_likelihood = new_likelihood;
        }

        VLOG(10) << "Done running expectation maximization...";
    }

    GaussianMixture EmAlgorithm::get_mixture() const {
        return mixture;
    }

    std::ostream& operator<<(std::ostream& os, const EmAlgorithm& em) {
        os << "===EM===" << std::endl << em.mixture;
        return os;
    }

    double EmAlgorithm::log_likelihood() {
        return pcl.log_likelihood_of_mixture(mixture);
    }
}
