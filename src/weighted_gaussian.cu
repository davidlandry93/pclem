
#include "weighted_gaussian.h"

namespace pclem {
    WeightedGaussian::WeightedGaussian() : mu(), sigma() {}

    WeightedGaussian::WeightedGaussian(Point& _mu, CovarianceMatrix& _sigma) :
        mu(_mu), sigma(_sigma) {
    }

    WeightedGaussian::WeightedGaussian(const WeightedGaussian& other) :
        mu(other.mu), sigma(other.sigma) {
    }

    CovarianceMatrix WeightedGaussian::get_sigma() const {
        CovarianceMatrix m(sigma);
        return m;
    }
}
