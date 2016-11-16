#ifndef VISUALIZABLE_GAUSSIAN_MIXTURE_H
#define VISUALIZABLE_GAUSSIAN_MIXTURE_H

#include "gaussian_mixture.h"

#include "visualization.h"

namespace pclem {
    class VisualizableGaussianMixture : public GaussianMixture {
    public:
        VisualizableGaussianMixture(const GaussianMixture& mixture) :
            GaussianMixture(mixture) {}
        void insert_into_visualization(Visualization& vis) const;
    private:
    };
}

#endif
