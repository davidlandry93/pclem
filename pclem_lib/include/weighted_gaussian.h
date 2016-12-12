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
        WeightedGaussian(const Point& mu, const CovarianceMatrix& sigma, double weight, double weight_in_hierarchy_of_parent);
        WeightedGaussian(const WeightedGaussian& other);

        Point get_mu() const;
        CovarianceMatrix get_sigma() const;
        double get_weight() const;
        double weight_in_hierarchy() const;

        friend std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g);

        void insert_into_visualization(Visualization& vis) const;

    private:
        Point mu;
        CovarianceMatrix sigma;
        double weight_in_mixture;
        double weight_in_hierarchy_of_parent;
    };
}

#endif
