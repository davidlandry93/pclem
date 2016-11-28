#ifndef VISUALIZABLE_HIERARCHICAL_GAUSSIAN_MIXTURE_H
#define VISUALIZABLE_HIERARCHICAL_GAUSSIAN_MIXTURE_H

#include "hierarchical_gaussian_mixture.h"
#include "visualization.h"

namespace pclem {
    class VisualizableHierarchicalGaussianMixture : public HierarchicalGaussianMixture {
    public:
        void insert_into_visualization(Visualization& vis) const;
    };
}

#endif
