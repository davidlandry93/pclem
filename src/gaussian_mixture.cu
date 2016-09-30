
#include "gaussian_mixture.h"


namespace pclem {
    GaussianMixture::GaussianMixture() :
        gaussians(std::vector<WeightedGaussian>(0)) {
    }

    GaussianMixture::GaussianMixture(std::vector<WeightedGaussian> _gaussians) :
        gaussians(_gaussians) {
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
