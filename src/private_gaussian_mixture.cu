
#include "private_gaussian_mixture.h"


namespace pclem {
    PrivateGaussianMixture::PrivateGaussianMixture() :
        gaussians(std::vector<WeightedGaussian>(0)) {
    }

    PrivateGaussianMixture::PrivateGaussianMixture(std::vector<WeightedGaussian> _gaussians) :
        gaussians(std::move(_gaussians)) {
    }

    PrivateGaussianMixture::PrivateGaussianMixture(PrivateGaussianMixture&& other) :
        gaussians(other.gaussians) {
    }

    std::vector<WeightedGaussian>::const_iterator PrivateGaussianMixture::begin() const {
        return gaussians.begin();
    }

    std::vector<WeightedGaussian>::const_iterator PrivateGaussianMixture::end() const {
        return gaussians.end();
    }

    int PrivateGaussianMixture::n_gaussians() const {
        return gaussians.size();
    }

    PrivateGaussianMixture& PrivateGaussianMixture::operator=(PrivateGaussianMixture&& other) {
        std::swap(gaussians, other.gaussians);
        return *this;
    }

    WeightedGaussian PrivateGaussianMixture::get_gaussian(int i) const {
        return gaussians[i];
    }

    std::ostream& operator<<(std::ostream& os, const PrivateGaussianMixture& mixture) {
        for(auto gaussian : mixture) {
            os << gaussian << std::endl;
        }
        os << std::endl;

        return os;
    }
}
