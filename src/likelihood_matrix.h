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
        GaussianMixture gaussian_mixture_of_pcl(const PointCloud& pcl) const;
        double log_likelihood(const PointCloud& pcl, const GaussianMixture& mixture) const;
        LikelihoodMatrix& operator=(LikelihoodMatrix&& other);
        void swap(LikelihoodMatrix& other);
    private:
        int n_distributions;
        int n_points;
        thrust::device_vector<double> likelihoods;

        LikelihoodMatrix(int n_points, int n_distributions, thrust::device_vector<double>& likelihoods);
        static void likelihoods_of_distribution(const PointCloud& pcl, const WeightedGaussian& distribution,
                                                thrust::device_vector<double>::iterator result);
        static void normalize_likelihoods(int n_points, thrust::device_vector<double>& likelihoods);
        Point compute_mu(const PointCloud& pcl,
                         thrust::device_vector<double>::const_iterator likelihoods,
                         double sum_of_gammas) const;
        CovarianceMatrix compute_sigma(const PointCloud& pcl,
                                       thrust::device_vector<double>::const_iterator likelihoods,
                                       double sum_of_gammas, const Point& new_mu) const;

        double log_likelihood_of_distribution(const PointCloud& pcl, const WeightedGaussian& distribution,
                                              thrust::device_vector<double>::const_iterator likelihoods) const;
    };
}

#endif
