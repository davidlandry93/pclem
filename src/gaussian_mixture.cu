
#include "gaussian_mixture.h"


namespace pclem {
    GaussianMixture::GaussianMixture() {
    }

    GaussianMixture::GaussianMixture(std::vector<WeightedGaussian> gaussians) :
        gaussians(gaussians) {
    }
}
