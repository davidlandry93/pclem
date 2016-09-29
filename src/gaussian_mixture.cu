
#include "gaussian_mixture.h"


namespace pclem {
    GaussianMixture::GaussianMixture() {
    }

    GaussianMixture::GaussianMixture(std::vector<WeightedGaussian> gaussians) :
        gaussians(gaussians) {
    }

    thrust::device_vector<WeightedGaussian>::const_iterator GaussianMixture::begin() const {
        return gaussians.begin();
    }

    thrust::device_vector<WeightedGaussian>::const_iterator GaussianMixture::end() const {
        return gaussians.end();
    }

    int GaussianMixture::n_gaussians() const {
        return gaussians.size();
    }
}
