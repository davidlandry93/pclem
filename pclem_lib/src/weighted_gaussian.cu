
#include <cmath>

#include "ellipsoid.h"
#include "weighted_gaussian.h"

namespace pclem {
    WeightedGaussian::WeightedGaussian() :mu(), sigma(), weight(0) {}

    WeightedGaussian::WeightedGaussian(const Point& mu, const CovarianceMatrix& sigma, double weight) :
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

    void WeightedGaussian::insert_into_visualization(Visualization& vis) const {
        CovarianceMatrix cov = get_sigma();

        auto eigen_result = cov.svd_decomposition();
        Vector3 eigenvalues = eigen_result.first;
        Matrix33 eigenvectors = eigen_result.second;

        std::cout << "Eigenvalues: " << eigenvalues[0] << eigenvalues[1] << eigenvalues[2] << std::endl;

        Vector3 position(get_mu().x, get_mu().y, get_mu().z);
        Ellipsoid ellipsoid(std::sqrt(eigenvalues[0]), std::sqrt(eigenvalues[1]), std::sqrt(eigenvalues[2]), position, eigenvectors, get_weight());

        std::cout << ellipsoid;

        vis.insert_ellipsoid(ellipsoid);
    }

    std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g) {
        os << "MU: " << g.get_mu() <<
            " WEIGHT: " << g.get_weight() <<
            " SIGMA: " << g.get_sigma();
        return os;
    }
}
