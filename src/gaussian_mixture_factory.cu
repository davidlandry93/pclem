
#include <glog/logging.h>

#include "point.h"
#include "gaussian_mixture_factory.h"

namespace pclem {

    GaussianMixture GaussianMixtureFactory::from_pcl_corners(const PointCloud& pcl) const {
        return from_pcl_corners(pcl.getBoundingBox());
    }

    GaussianMixture GaussianMixtureFactory::from_pcl_corners(const DevicePointCloud& pcl) const {
        return from_pcl_corners(pcl.getBoundingBox());
    }

    GaussianMixture GaussianMixtureFactory::from_pcl_corners(const BoundingBox& bounding_box) const {
        auto corners = bounding_box.corners();
        double initial_weight_of_gaussian = 1.0 / corners.size();

        std::vector<WeightedGaussian> temp_gaussians;
        for(Point corner : corners) {
            CovarianceMatrix sigma = CovarianceMatrix();
            sigma.set(0,0,10.0);
            sigma.set(1,1,10.0);
            sigma.set(2,2,10.0);

            WeightedGaussian gaussian(corner, sigma, initial_weight_of_gaussian);
            VLOG(4) << "Adding gaussian: " << gaussian;
            temp_gaussians.push_back(gaussian);
        }

        GaussianMixture mixture(temp_gaussians);
        return mixture;
    }


}
