
#include <array>

#include "gtest/gtest.h"

#include "point.h"
#include "covariance_matrix.h"
#include "weighted_gaussian.h"
#include "pointcloud.h"
#include "vtk_visualization.h"

namespace pclem{

    TEST(VtkVisualizerTest, WeightedGaussianPositionTest) {
        Point mu(0.0, 0.0, 0.0);
        std::array<double,9> cov_data = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
        CovarianceMatrix sigma(cov_data);

        WeightedGaussian gaussian(mu, sigma, 1.0, 1.0);

        Ellipsoid e(1.0, 1.0, 1.0, Vector3(0.0, 0.0, 0.0), Matrix3::)

        VtkVisualization vis;
        gaussian.insert_into_visualization(vis);

        vis.visualize();
    }

    TEST(VtkVisualizerTest, PointGridTest) {
        PointCloud pointcloud;

        std::vector<Point> points;
        for(int i=0; i < 10; i++) {
            for(int j=0; j < 10; j++) {
                for(int k=0; k < 10; k++) {
                    points.push_back(Point(i * 1.0, j * 1.0, k * 1.0));
                }
            }
        }

        pointcloud.set_points(points);

        VtkVisualization vis;
        pointcloud.insert_into_visualization(vis);
        vis.visualize();
    }

}
