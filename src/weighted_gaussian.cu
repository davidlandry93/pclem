
#include "weighted_gaussian.h"


namespace pclem {
    WeightedGaussian::WeightedGaussian(Point& _mu, CovarianceMatrix& _sigma) :
        mu(_mu), sigma(_sigma) {
    }

    CovarianceMatrix WeightedGaussian::get_sigma() const {
        CovarianceMatrix m(sigma);
        return m;
    }

    Point WeightedGaussian::get_mu() const {
        Point p(mu);
        return p;
    }

    std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g) {
        os << "MU: " << g.get_mu() << "SIGMA: "
           << g.get_sigma() << std::endl;
        return os;
    }
}
