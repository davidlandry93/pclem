
#include <cstdlib>
#include <ctime>
#include <glog/logging.h>

#include "point.h"
#include "gaussian_mixture_factory.h"

namespace pclem {
    bool GaussianMixtureFactory::random_seeded = false;

    GaussianMixtureFactory::GaussianMixtureFactory() {
        if(!random_seeded) {
            srand(static_cast <unsigned> (time(0)));
        }
    }

    GaussianMixture GaussianMixtureFactory::from_pcl_corners(const PointCloud& pcl, double weight_of_parent_in_hierarchy) const {
        return from_pcl_corners(pcl.getBoundingBox(), weight_of_parent_in_hierarchy);
    }

    GaussianMixture GaussianMixtureFactory::from_pcl_corners(const DevicePointCloud& pcl, double weight_of_parent_in_hierarchy) const {
        return from_pcl_corners(pcl.getBoundingBox(), weight_of_parent_in_hierarchy);
    }

    GaussianMixture GaussianMixtureFactory::from_pcl_corners(const BoundingBox& bounding_box, double weight_of_parent_in_hierarchy) const {
        auto corners = bounding_box.corners();
        double initial_weight_of_gaussian = 1.0 / corners.size();

        std::vector<WeightedGaussian> temp_gaussians;
        for(Point corner : corners) {
            CovarianceMatrix sigma = CovarianceMatrix();
            sigma.set(0,0,10.0);
            sigma.set(1,1,10.0);
            sigma.set(2,2,10.0);

            WeightedGaussian gaussian(corner, sigma, initial_weight_of_gaussian, weight_of_parent_in_hierarchy);
            VLOG(4) << "Adding gaussian: " << gaussian;
            temp_gaussians.push_back(gaussian);
        }

        GaussianMixture mixture(temp_gaussians);
        return mixture;
    }

    GaussianMixture GaussianMixtureFactory::around_point(const Point& point, const CovarianceMatrix& cov,
                                                         int n_of_distributions, double delta, double weight_of_parent_in_hierarchy) const {
        VLOG(10) << "Creating gaussian mixture around point...";

        CovarianceMatrix sigma;
        sigma.set(0,0,1.0);
        sigma.set(1,1,1.0);
        sigma.set(2,2,1.0);

        std::vector<WeightedGaussian> temp_gaussians;

        for(int i = 0; i < n_of_distributions; i++) {
            WeightedGaussian gaussian(
                Point(point.x + random_number(-delta, delta),
                      point.y + random_number(-delta, delta),
                      point.z + random_number(-delta, delta)),
                sigma,
                1.0 / n_of_distributions,
                weight_of_parent_in_hierarchy);

            temp_gaussians.push_back(gaussian);
        }

        GaussianMixture resulting_mixture(temp_gaussians);

        VLOG(10) << "Done creating gaussian mixture.";
        return resulting_mixture;
    }

    double GaussianMixtureFactory::random_number(double min, double max) {
        return min + static_cast<double> (rand()) / static_cast<double> (RAND_MAX/(max-min));
    }
}
