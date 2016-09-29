
#include "gaussian_mixture.h"


namespace pclem {
    GaussianMixture::GaussianMixture() {
    }

    GaussianMixture::GaussianMixture(std::vector<WeightedGaussian> gaussians) :
        gaussians(gaussians) {
    }

    const thrust::device_vector<WeightedGaussian>& GaussianMixture::get_gaussians() const {
        return gaussians;
    }

    int GaussianMixture::n_gaussians() const {
        return gaussians.size();
    }
}
