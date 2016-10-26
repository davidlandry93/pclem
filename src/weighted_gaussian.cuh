#ifndef WEIGHTED_GAUSSIAN_H
#define WEIGHTED_GAUSSIAN_H

#include <iostream>

#include "point.h"
#include "covariance_matrix.cuh"

namespace pclem {
    class WeightedGaussian {
    public:
        WeightedGaussian() : mu(), sigma(), weight(0.0) {}
        WeightedGaussian(Point& mu, CovarianceMatrix& sigma, double weight) :
            mu(mu), sigma(sigma), weight(weight) {}

        WeightedGaussian(const WeightedGaussian& other) :
            mu(other.mu), sigma(other.sigma), weight(other.weight) {}

        Point get_mu() const;
        CovarianceMatrix get_sigma() const;
        double get_weight() const;

        friend std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g);

    private:
        Point mu;
        CovarianceMatrix sigma;
        double weight;
    };
}

#endif
