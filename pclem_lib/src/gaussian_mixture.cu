
#include "gaussian_mixture.h"


namespace pclem {
    GaussianMixture::GaussianMixture() :
        gaussians() {
    }

    GaussianMixture::GaussianMixture(const std::vector<WeightedGaussian>& _gaussians) :
        gaussians(_gaussians) {
    }

    GaussianMixture::GaussianMixture(const GaussianMixture& other) :
        gaussians(other.gaussians) {
    }

    std::vector<WeightedGaussian>::const_iterator GaussianMixture::begin() const {
        return gaussians.begin();
    }

    std::vector<WeightedGaussian>::const_iterator GaussianMixture::end() const {
        return gaussians.end();
    }

    int GaussianMixture::n_gaussians() const {
        return gaussians.size();
    }

    WeightedGaussian GaussianMixture::get_gaussian(int i) const {
        return gaussians[i];
    }

    std::ostream& operator<<(std::ostream& os, const GaussianMixture& mixture) {
        for(auto gaussian : mixture) {
            os << gaussian << std::endl;
        }
        os << std::endl;

        return os;
    }
}
