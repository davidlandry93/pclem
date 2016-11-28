
#include "visualizable_weighted_gaussian.h"

namespace pclem {
    
    void VisualizableHierarchicalGaussianMixture::insert_into_visualization(Visualization& vis) const {
        std::vector<WeightedGaussian> leaves;
        device_mixture->get_leaves(leaves);

        for(WeightedGaussian leave: leaves) {
            Visua
        }
    }
}
