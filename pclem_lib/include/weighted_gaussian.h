#ifndef WEIGHTED_GAUSSIAN_H
#define WEIGHTED_GAUSSIAN_H

#include <iostream>

#include "point.h"
#include "vector3.h"
#include "matrix33.h"
#include "visualization.h"

namespace pclem {
    class WeightedGaussian {
    public:
        WeightedGaussian();
        WeightedGaussian(const Point& mu, const Matrix33& sigma, double weight);
        WeightedGaussian(const Point& mu, const Matrix33& sigma, double weight, double weight_in_hierarchy_of_parent);
        WeightedGaussian(const WeightedGaussian& other);

        Point get_mu() const;
        Matrix33 get_sigma() const;
        double get_weight() const;
        double weight_in_hierarchy() const;

        std::pair<Vector3, Matrix33> eigen_decomposition() const;

        friend std::ostream &operator<<(std::ostream &os, WeightedGaussian const &g);

        void insert_into_visualization(Visualization& vis) const;

    private:
        Point mu;
        Matrix33 sigma;
        double weight_in_mixture;
        double weight_in_hierarchy_of_parent;
    };
}

#endif
