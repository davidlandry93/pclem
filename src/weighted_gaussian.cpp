
#include "weighted_gaussian.h"

namespace pclem {
    WeightedGaussian::WeightedGaussian() : mu(), sigma() {}

    WeightedGaussian::WeightedGaussian(Point& mu, CovarianceMatrix& sigma) :
        mu(std::move(mu)), sigma(std::move(sigma)) {
    }

    CovarianceMatrix WeightedGaussian::get_sigma() {
        CovarianceMatrix m(sigma);
        return m;
    }
}
