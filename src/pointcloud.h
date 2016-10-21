#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>
#include <vtkPolyData.h>

#include <thrust/device_vector.h>

#include "point.cuh"
#include "associated_point.cuh"
#include "boundingbox.h"
#include "weighted_gaussian.cuh"
#include "gaussian_mixture.h"

namespace pclem {

    class PointCloud {
    public:
        static PointCloud from_vtk(vtkPolyData* vtkData);
        PointCloud(PointCloud&& other);
        PointCloud& operator=(PointCloud&& other);
        BoundingBox getBoundingBox();
        int get_n_points() const;
        thrust::device_vector<AssociatedPoint>::const_iterator begin() const;
        thrust::device_vector<AssociatedPoint>::const_iterator end() const;
        void compute_associations(const GaussianMixture& mixture);
        void normalize_associations();
        GaussianMixture create_mixture() const;
        double log_likelihood_of_mixture(const GaussianMixture& mixture) const;
    private:
        thrust::device_vector<AssociatedPoint> data;
        int n_points;
        BoundingBox boundingBox;

        PointCloud(std::vector<AssociatedPoint> data);
        PointCloud(PointCloud& other);
        void updateBoundingBox();
        void normalize_likelihoods(thrust::device_vector<double>& likelihoods, int n_gaussians, int n_points) const;
        void compute_associations_of_distribution(int index_of_distribution, const WeightedGaussian& distribution);
        WeightedGaussian create_distribution_of_mixture(int index, double sum_of_gammas) const;
        CovarianceMatrix compute_sigma(int index, const Point& mu, double sum_of_gammas) const;
        double log_likelihood_of_distribution(int index_of_distribution, const WeightedGaussian& distribution) const;
    };

}

#endif
