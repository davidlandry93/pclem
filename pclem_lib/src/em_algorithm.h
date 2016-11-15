#ifndef EM_ALGORITHM_H
#define EM_ALGORITHM_H

#include <memory>

#include "pointcloud.h"
#include "gaussian_mixture.h"

namespace pclem {
    class EmAlgorithm {
    public:
        EmAlgorithm(EmAlgorithm&& other);
        static EmAlgorithm from_pcl(PointCloud& pcl);
        void expectation();
        void maximization();
        void run(double epsilon);
        double log_likelihood();
        EmAlgorithm& operator=(EmAlgorithm&& other);
        GaussianMixture get_mixture() const;
        friend std::ostream& operator<<(std::ostream& os, const EmAlgorithm& em);
    private:
        EmAlgorithm(PointCloud& pcl, GaussianMixture& mixture);
        PointCloud pcl;
        GaussianMixture mixture;
    };
}

#endif
