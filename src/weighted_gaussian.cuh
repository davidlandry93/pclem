#ifndef WEIGHTED_GAUSSIAN_H
#define WEIGHTED_GAUSSIAN_H

#include <iostream>

#include "covariance_matrix.cuh"
#include "point.cuh"

namespace pclem {
    class WeightedGaussian {
    public:
        __host__ __device__
        WeightedGaussian() : mu(), sigma() {}
        WeightedGaussian(Point& mu, CovarianceMatrix& sigma);

        __host__ __device__
        WeightedGaussian(const WeightedGaussian& other) :
            mu(other.mu), sigma(other.sigma) {}

        Point get_mu() const;
        CovarianceMatrix get_sigma() const;

        friend std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g);

    private:
        Point mu;
        CovarianceMatrix sigma;
    };
}

#endif
