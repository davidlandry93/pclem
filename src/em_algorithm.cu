
#include "em_algorithm.h"
#include "covariance_matrix.cuh"

namespace pclem {
    EmAlgorithm::EmAlgorithm(PointCloud& pcl,
                             GaussianMixture& mixture,
                             std::vector<double>& likelihoods) :
        pcl(std::move(pcl)),
        mixture(std::move(mixture)),
        likelihoods(pcl.get_n_points(), mixture.n_gaussians(), likelihoods) {
    }

    EmAlgorithm::EmAlgorithm(EmAlgorithm&& other) :
        pcl(std::move(other.pcl)), mixture(std::move(other.mixture)),
        likelihoods(std::move(other.likelihoods)) {}

    EmAlgorithm EmAlgorithm::from_pcl(PointCloud& pcl) {
        std::vector<WeightedGaussian> temp_gaussians;
        for(Point corner : pcl.getBoundingBox().corners()) {
            CovarianceMatrix sigma = CovarianceMatrix();
            sigma.set(0,0,10.0);
            sigma.set(1,1,10.0);
            sigma.set(2,2,10.0);

            WeightedGaussian gaussian(corner, sigma);
            temp_gaussians.push_back(gaussian);
        }

        GaussianMixture mixture(temp_gaussians);

        auto likelihoods = std::vector<double>(mixture.n_gaussians()*pcl.get_n_points());

        EmAlgorithm temp_em(pcl, mixture, likelihoods);

        return temp_em;
    }

    EmAlgorithm& EmAlgorithm::operator=(EmAlgorithm&& other) {
        std::swap(pcl, other.pcl);
        std::swap(mixture, other.mixture);
        std::swap(likelihoods, other.likelihoods);
        return *this;
    }

    void EmAlgorithm::expectation() {
        likelihoods = LikelihoodMatrix::build(pcl, mixture);
    }

    void EmAlgorithm::maximization() {
        GaussianMixture new_mixture = likelihoods.gaussian_mixture_of_pcl(pcl);
    }

    std::ostream& operator<<(std::ostream& os, const EmAlgorithm& em) {
        std::cout << "EM: " << em.mixture.get_gaussian(0);
        return os;
    }
}
