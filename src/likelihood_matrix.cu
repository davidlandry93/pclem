
#include "likelihood_matrix.h"

namespace pclem {
    LikelihoodMatrix::LikelihoodMatrix(thrust::device_vector<double> likelihoods)
        : likelihoods(likelihoods) {}

    LikelihoodMatrix LikelihoodMatrix::build(const PointCloud& pcl, const GaussianMixture& mixture) {
        thrust::device_vector<double> likelihoods;

        return LikelihoodMatrix(likelihoods);
    }
}
