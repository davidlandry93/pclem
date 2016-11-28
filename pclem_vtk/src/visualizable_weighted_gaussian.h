#ifndef VISUALIZABLE_WEIGHTED_GAUSSIAN_H
#define VISUALIZABLE_WEIGHTED_GAUSSIAN_H

#include "weighted_gaussian.h"
#include "visualization.h"

namespace pclem {
    class VisualizableWeightedGaussian : public WeightedGaussian {
    public:
        VisualizableWeightedGaussian(const WeightedGaussian& gaussian);
        void insert_into_visualization(Visualization& vis) const;
    };
}

#endif
