
#include <glog/logging.h>

#include "device_pointcloud.h"
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
        GaussianMixture mixture = gm_factory.from_pcl_corners(pcl, 1.0);

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

        std::cout << "Running em..." << std::endl;
        std::cout << "PCL has " << pcl.get_n_points() << " points" << std::endl;
        std::cout << "Initial mixture: " << std::endl << mixture << std::endl;

        double previous_likelihood = 0.0;
        double delta = std::numeric_limits<double>::infinity();
        int n_iterations = 0;

        while((delta > epsilon ||
               previous_likelihood == -1*std::numeric_limits<double>::infinity() ||
               n_iterations < MIN_N_ITERATIONS) &&
              n_iterations < MAX_N_ITERATIONS) {
            expectation();
            maximization();

            double new_likelihood = log_likelihood();
            delta = std::abs(new_likelihood - previous_likelihood);
            VLOG(3) << "Log likelihood: " << new_likelihood;
            VLOG(3) << "Delta: " << delta;

            previous_likelihood = new_likelihood;

            n_iterations++;
        }

        std::cout << "Result: " << std::endl << mixture;

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
