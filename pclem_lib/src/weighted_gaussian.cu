
#include <cmath>

#include "ellipsoid.h"
#include "weighted_gaussian.h"

namespace pclem {
    WeightedGaussian::WeightedGaussian() :mu(), sigma(), weight_in_mixture(0.0), weight_in_hierarchy_of_parent(0.0) {}

    WeightedGaussian::WeightedGaussian(const Point& mu, const Matrix33& sigma, double weight_in_mixture) :
        WeightedGaussian(mu, sigma, weight_in_mixture, 1.0) {}

    WeightedGaussian::WeightedGaussian(const Point& mu, const Matrix33& sigma, double weight_in_mixture, double weight_in_hierarchy_of_parent) :
        mu(mu), sigma(sigma), weight_in_mixture(weight_in_mixture), weight_in_hierarchy_of_parent(weight_in_hierarchy_of_parent) {}

    WeightedGaussian::WeightedGaussian(const WeightedGaussian& other) :
        mu(other.mu), sigma(other.sigma), weight_in_mixture(other.weight_in_mixture),
        weight_in_hierarchy_of_parent(other.weight_in_hierarchy_of_parent) {}

    Matrix33 WeightedGaussian::get_sigma() const {
        Matrix33 m(sigma);
        return m;
    }

    Point WeightedGaussian::get_mu() const {
        Point p(mu);
        return p;
    }

    double WeightedGaussian::get_weight() const {
        return weight_in_mixture;
    }

    double WeightedGaussian::weight_in_hierarchy() const {
        return weight_in_mixture * weight_in_hierarchy_of_parent;
    }

    void WeightedGaussian::insert_into_visualization(Visualization& vis) const {
        Matrix33 cov = get_sigma();

        auto eigen_result = cov.eigen_decomposition();
        Vector3 eigenvalues = eigen_result.first;
        Matrix33 eigenvectors = eigen_result.second;

        Vector3 position(get_mu().x, get_mu().y, get_mu().z);
        Ellipsoid ellipsoid(std::sqrt(eigenvalues[0]), std::sqrt(eigenvalues[1]), std::sqrt(eigenvalues[2]), position, eigenvectors, weight_in_hierarchy());

        vis.insert_ellipsoid(ellipsoid);
    }

    std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g) {
        os << " WEIGHT: " << g.get_weight() <<
            " W IN HIERARCHY: " << g.weight_in_hierarchy() <<
            " MU: " << g.get_mu() <<
            " SIGMA: " << g.get_sigma();
        return os;
    }
}
