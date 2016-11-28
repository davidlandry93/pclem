
#include <math.h>
#include <cmath>

#include "vector3.h"
#include "covariance_matrix.h"
#include "weighted_gaussian.h"

#include "ellipsoid.h"
#include "visualizable_weighted_gaussian.h"
#include "visualizable_gaussian_mixture.h"

namespace pclem {

    void VisualizableGaussianMixture::insert_into_visualization(Visualization& vis) const {
        for(WeightedGaussian gaussian : gaussians) {
            auto visualizable_gaussian = VisualizableWeightedGaussian(gaussian);
            visualizable_gaussian.insert_into_visualization(vis);
        }
    }
}
