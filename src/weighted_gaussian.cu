
#include "weighted_gaussian.h"


namespace pclem {
    WeightedGaussian::WeightedGaussian() :mu(), sigma(), weight(0) {}

    WeightedGaussian::WeightedGaussian(Point& mu, CovarianceMatrix& sigma, double weight) :
        mu(mu), sigma(sigma), weight(weight) {}

    WeightedGaussian::WeightedGaussian(const WeightedGaussian& other) :
        mu(other.mu), sigma(other.sigma), weight(other.weight) {}

    CovarianceMatrix WeightedGaussian::get_sigma() const {
        CovarianceMatrix m(sigma);
        return m;
    }

    Point WeightedGaussian::get_mu() const {
        Point p(mu);
        return p;
    }

    double WeightedGaussian::get_weight() const {
        return weight;
    }

    std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g) {
        os << "MU: " << g.get_mu() <<
            " WEIGHT: " << g.get_weight() <<
            " SIGMA: " << g.get_sigma() << std::endl;
        return os;
    }
}
