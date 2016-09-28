
#include "em_algorithm.h"
#include "covariance_matrix.h"

namespace pclem {
    EmAlgorithm::EmAlgorithm(PointCloud pcl) :
        pcl(std::move(pcl)), mixture() {

        std::vector<WeightedGaussian> temp_gaussians;
        for(Point corner : pcl.getBoundingBox().corners()) {
            CovarianceMatrix sigma = CovarianceMatrix();
            sigma.set(0,0,1.0);
            sigma.set(1,1,1.0);
            sigma.set(2,2,1.0);


            WeightedGaussian gaussian(corner, sigma);
            temp_gaussians.push_back(gaussian);
        }
    }
}
