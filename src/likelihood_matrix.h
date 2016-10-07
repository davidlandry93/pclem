#ifndef LIKELIHOOD_MATRIX_H
#define LIKELIHOOD_MATRIX_H

#include <vector>
#include <thrust/device_vector.h>

#include "pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class LikelihoodMatrix {
    public:
        LikelihoodMatrix();
        LikelihoodMatrix(int n_points, int n_distributions, std::vector<double>& likelihoods);
        LikelihoodMatrix(LikelihoodMatrix&& other);
        static LikelihoodMatrix build(const PointCloud& pcl, const GaussianMixture& mixture);
        LikelihoodMatrix& operator=(LikelihoodMatrix&& other);
        void swap(LikelihoodMatrix& other);
    private:
        int n_distributions;
        int n_points;
        thrust::device_vector<double> likelihoods;

        LikelihoodMatrix(int n_points, int n_distributions, thrust::device_vector<double>& likelihoods);
        static void likelihoods_of_distribution(const PointCloud& pcl, const WeightedGaussian& distribution, thrust::device_vector<double>::iterator result);
    };
}

#endif
