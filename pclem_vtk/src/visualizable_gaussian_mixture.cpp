
#include <math.h>
#include <cmath>

#include "vector3.h"
#include "covariance_matrix.h"
#include "weighted_gaussian.h"

#include "ellipsoid.h"
#include "visualizable_gaussian_mixture.h"

namespace pclem {

    void VisualizableGaussianMixture::insert_into_visualization(Visualization& vis) const {
        for(WeightedGaussian gaussian : gaussians) {
            CovarianceMatrix cov = gaussian.get_sigma();

            auto eigen_result = cov.eigenvalues();
            Vector3 eigenvalues = eigen_result.first;
            Vector3 first_eigenvector = eigen_result.second[0];
            Vector3 second_eigenvector = eigen_result.second[1];

            std::cout << "Eigenvalues: " << eigenvalues[0] << eigenvalues[1] << eigenvalues[2] << std::endl;

            Vector3 rotation;

            rotation.x = atan(first_eigenvector[1] / first_eigenvector[2]);
            rotation.y = atan(first_eigenvector[2] / first_eigenvector[0]);
            rotation.z = atan(first_eigenvector[1] / first_eigenvector[0]);

            Vector3 position(gaussian.get_mu().x, gaussian.get_mu().y, gaussian.get_mu().z);

            Ellipsoid ellipsoid(std::sqrt(eigenvalues[0]), std::sqrt(eigenvalues[1]), std::sqrt(eigenvalues[2]), position, rotation);

            std::cout << ellipsoid;

            vis.insert_ellipsoid(ellipsoid);
        }
    }
}
