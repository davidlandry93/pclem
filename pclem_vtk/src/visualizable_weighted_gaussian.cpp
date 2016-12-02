
#include <cmath>
#include <armadillo>

#include "vector3.h"
#include "covariance_matrix.h"
#include "visualizable_weighted_gaussian.h"
#include "ellipsoid.h"


namespace pclem {
    VisualizableWeightedGaussian::VisualizableWeightedGaussian(const WeightedGaussian& gaussian) :
        WeightedGaussian(gaussian) {}

    void VisualizableWeightedGaussian::insert_into_visualization(Visualization& vis) const {
        CovarianceMatrix cov = get_sigma();

        auto eigen_result = cov.svd_decomposition();
        Vector3 eigenvalues = eigen_result.first;
        Matrix33 eigenvectors = eigen_result.second;

        std::cout << "Eigenvalues: " << eigenvalues[0] << eigenvalues[1] << eigenvalues[2] << std::endl;

        Vector3 position(get_mu().x, get_mu().y, get_mu().z);
        Ellipsoid ellipsoid(std::sqrt(eigenvalues[0]), std::sqrt(eigenvalues[1]), std::sqrt(eigenvalues[2]), position, eigenvectors, get_weight());

        std::cout << ellipsoid;

        vis.insert_ellipsoid(ellipsoid);
    }
}
