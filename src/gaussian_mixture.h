#ifndef WEIGHTED_GAUSSIANS_H
#define WEIGHTED_GAUSSIANS_H

#include <vector>
#include <iostream>

#include "weighted_gaussian.h"

namespace pclem {
    class GaussianMixture {
    public:
        GaussianMixture();
        GaussianMixture(std::vector<WeightedGaussian> gaussians);
        GaussianMixture(GaussianMixture&& other);
        GaussianMixture& operator=(GaussianMixture&& other);
        GaussianMixture(const GaussianMixture& other);
        std::vector<WeightedGaussian>::const_iterator begin() const;
        std::vector<WeightedGaussian>::const_iterator end() const;
        int n_gaussians() const;
        WeightedGaussian get_gaussian(int i) const; 
        friend std::ostream& operator<<(std::ostream& os, const GaussianMixture& mixture);

    private:
        std::vector<WeightedGaussian> gaussians;
    };
}

#endif
