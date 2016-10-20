#ifndef EM_ALGORITHM_H
#define EM_ALGORITHM_H

#include <memory>

#include <thrust/device_vector.h>

#include "pointcloud.h"
#include "gaussian_mixture.h"
#include "likelihood_matrix.h"

namespace pclem {
    class EmAlgorithm {
    public:
        static const int N_DISTRIBUTIONS_PER_MIXTURE = 8;

        EmAlgorithm(EmAlgorithm&& other);
        static EmAlgorithm from_pcl(PointCloud& pcl);
        void expectation();
        void maximization();
        double log_likelihood();
        EmAlgorithm& operator=(EmAlgorithm&& other);
        friend std::ostream& operator<<(std::ostream& os, const EmAlgorithm& em);
    private:
        EmAlgorithm(PointCloud& pcl, GaussianMixture& mixture, std::vector<double>& likelihoods);
        PointCloud pcl;
        GaussianMixture mixture;
        LikelihoodMatrix likelihoods;
    };
}

#endif
