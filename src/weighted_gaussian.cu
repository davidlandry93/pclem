
#include "weighted_gaussian.cuh"


namespace pclem {
    WeightedGaussian::WeightedGaussian(Point& _mu, CovarianceMatrix& _sigma, double _weight) :
        mu(_mu), sigma(_sigma), weight(_weight) {
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
