#ifndef WEIGHTED_GAUSSIANS_H
#define WEIGHTED_GAUSSIANS_H

#include <vector>
#include <iostream>

#include "visualization.h"
#include "weighted_gaussian.h"

namespace pclem {
    class GaussianMixture {
    public:
        GaussianMixture();
        GaussianMixture(const std::vector<WeightedGaussian>& gaussians);
        GaussianMixture(const GaussianMixture& other);
        std::vector<WeightedGaussian>::const_iterator begin() const;
        std::vector<WeightedGaussian>::const_iterator end() const;
        int n_gaussians() const;
        int n_nonzero_gaussians() const;
        WeightedGaussian get_gaussian(int i) const; 
        friend std::ostream& operator<<(std::ostream& os, const GaussianMixture& mixture);
        void insert_into_visualization(Visualization& vis) const;

    protected:
        std::vector<WeightedGaussian> gaussians;
    };
}

#endif
