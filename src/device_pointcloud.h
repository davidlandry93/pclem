#ifndef DEVICE_POINTCLOUD_H
#define DEVICE_POINTCLOUD_H

#include <memory>
#include <vector>

#include <thrust/device_vector.h>

#include "point.h"
#include "boundingbox.h"
#include "weighted_gaussian.h"
#include "gaussian_mixture.h"
#include "associated_point.cuh"
#include "device_hierarchical_gaussian_mixture.h"
#include "hierarchical_gaussian_mixture.h"

namespace pclem {

    class DevicePointCloud {
    public:
        DevicePointCloud();
        DevicePointCloud(DevicePointCloud& other);
        DevicePointCloud(DevicePointCloud&& other);
        DevicePointCloud& operator=(DevicePointCloud&& other);
        BoundingBox getBoundingBox() const;
        int get_n_points() const;
        void compute_associations(const GaussianMixture& mixture);
        void normalize_associations();
        GaussianMixture create_mixture() const;
        double log_likelihood_of_mixture(const GaussianMixture& mixture) const;
        void add_points(std::vector<Point> points);
        std::vector<Point> copy_of_points() const;
        HierarchicalGaussianMixture create_hgmm();

    private:
        thrust::device_vector<AssociatedPoint> data;
        BoundingBox boundingBox;

        DevicePointCloud(std::vector<AssociatedPoint> data);
        void updateBoundingBox();
        void normalize_likelihoods(thrust::device_vector<double>& likelihoods, int n_gaussians, int n_points) const;
        void compute_associations_of_distribution(int index_of_distribution, const WeightedGaussian& distribution);
        WeightedGaussian create_distribution_of_mixture(int index, double sum_of_gammas) const;
        CovarianceMatrix compute_sigma(int index, const Point& mu, double sum_of_gammas) const;
        double log_likelihood_of_distribution(int index_of_distribution, const WeightedGaussian& distribution) const;
    };

}

#endif
