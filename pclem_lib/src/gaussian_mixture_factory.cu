
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

        Matrix33 sigma = covariance_from_pcl_corners(bounding_box);

        std::vector<WeightedGaussian> temp_gaussians;
        for(Point corner : corners) {
            WeightedGaussian gaussian(corner, sigma, initial_weight_of_gaussian, weight_of_parent_in_hierarchy);
            VLOG(4) << "Adding gaussian: " << gaussian;
            temp_gaussians.push_back(gaussian);
        }

        GaussianMixture mixture(temp_gaussians);
        return mixture;
    }

    // A heuristic to make the initial covariance matrix proportional to the size
    // of the bounding box of the point cloud.
    Matrix33 GaussianMixtureFactory::covariance_from_pcl_corners(const BoundingBox& bounding_box) const {
        Matrix33 m = Matrix33::zeros();

        Point min = bounding_box.getMin();
        Point max = bounding_box.getMax();

        double dx = max.x - min.x;
        double dy = max.y - min.y;
        double dz = max.z - min.z;

        m.set_element(0,0, (dx*dx/16));
        m.set_element(1,1, (dy*dy/16));
        m.set_element(2,2, (dz*dz/16));

        VLOG(1) << "Initial variances. X: " << m.get_element(0,0) << "Y: " << m.get_element(1,1) << "Z: " << m.get_element(2,2);

        return m;
    }

    GaussianMixture GaussianMixtureFactory::around_point(const Point& point, const Matrix33& cov,
                                                         int n_of_distributions, double delta, double weight_of_parent_in_hierarchy) const {
        VLOG(10) << "Creating gaussian mixture around point...";

        Matrix33 sigma;
        sigma.set_element(0,0,1.0);
        sigma.set_element(1,1,1.0);
        sigma.set_element(2,2,1.0);

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
