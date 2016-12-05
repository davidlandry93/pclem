#ifndef WEIGHTED_GAUSSIAN_H
#define WEIGHTED_GAUSSIAN_H

#include <iostream>

#include "point.h"
#include "covariance_matrix.h"
#include "visualization.h"

namespace pclem {
    class WeightedGaussian {
    public:
        WeightedGaussian();
        WeightedGaussian(const Point& mu, const CovarianceMatrix& sigma, double weight);
        WeightedGaussian(const WeightedGaussian& other);

        Point get_mu() const;
        CovarianceMatrix get_sigma() const;
        double get_weight() const;

        friend std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g);

        void insert_into_visualization(Visualization& vis) const;

    private:
        Point mu;
        CovarianceMatrix sigma;
        double weight;
    };
}

#endif
