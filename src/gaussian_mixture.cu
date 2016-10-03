
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

        std::cout << other.gaussians[0];
        std::cout << "During construction: " << gaussians[0];
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


    GaussianMixture& GaussianMixture::operator=(GaussianMixture&& other) {
        std::swap(gaussians, other.gaussians);
        return *this;
    }

    WeightedGaussian GaussianMixture::get_gaussian(int i) const {
        return gaussians[i];
    }
}
