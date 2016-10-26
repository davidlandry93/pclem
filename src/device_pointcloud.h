#ifndef DEVICE_POINTCLOUD_H
#define DEVICE_POINTCLOUD_H

#include <memory>
#include <vector>

#include <thrust/device_vector.h>

#include "point.h"
#include "boundingbox.h"
#include "weighted_gaussian.cuh"
#include "private_gaussian_mixture.h"
#include "associated_point.cuh"

namespace pclem {

    class DevicePointCloud {
    public:
        DevicePointCloud();
        DevicePointCloud(DevicePointCloud&& other);
        DevicePointCloud& operator=(DevicePointCloud&& other);
        BoundingBox getBoundingBox();
        int get_n_points() const;
        void compute_associations(const PrivateGaussianMixture& mixture);
        void normalize_associations();
        PrivateGaussianMixture create_mixture() const;
        double log_likelihood_of_mixture(const PrivateGaussianMixture& mixture) const;
        void add_points(std::vector<AssociatedPoint> points);
    private:
        thrust::device_vector<AssociatedPoint> data;
        BoundingBox boundingBox;

        DevicePointCloud(std::vector<AssociatedPoint> data);
        DevicePointCloud(DevicePointCloud& other);
        void updateBoundingBox();
        void normalize_likelihoods(thrust::device_vector<double>& likelihoods, int n_gaussians, int n_points) const;
        void compute_associations_of_distribution(int index_of_distribution, const WeightedGaussian& distribution);
        WeightedGaussian create_distribution_of_mixture(int index, double sum_of_gammas) const;
        CovarianceMatrix compute_sigma(int index, const Point& mu, double sum_of_gammas) const;
        double log_likelihood_of_distribution(int index_of_distribution, const WeightedGaussian& distribution) const;
    };

}

#endif
