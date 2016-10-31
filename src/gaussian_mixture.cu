
#include "gaussian_mixture.h"


namespace pclem {
    GaussianMixture::GaussianMixture() :
        gaussians(std::vector<WeightedGaussian>(0)) {
    }

    GaussianMixture::GaussianMixture(std::vector<WeightedGaussian> _gaussians) :
        gaussians(std::move(_gaussians)) {
    }

    GaussianMixture::GaussianMixture(GaussianMixture&& other) :
        gaussians(other.gaussians) {
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

    GaussianMixture& GaussianMixture::operator=(GaussianMixture&& other) {
        std::swap(gaussians, other.gaussians);
        return *this;
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
