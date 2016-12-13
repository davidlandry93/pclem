
#include <cmath>
#include <array>

#include "gtest/gtest.h"

#include "point.h"
#include "matrix33.h"
#include "weighted_gaussian.h"
#include "pointcloud.h"
#include "vtk_visualization.h"

#define PI 3.1416

namespace pclem{

    TEST(VtkVisualizerTest, WeightedGaussianPositionTest) {
        Point mu(0.0, 0.0, 0.0);
        Matrix33 sigma({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        Ellipsoid e(1.0, 1.0, 1.0, Vector3(0.0, 0.0, 0.0), Matrix33::identity(), 0.8);

        VtkVisualization vis;
        vis.insert_ellipsoid(e);

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

    TEST(VtkVisualizerTest, TranslationThenRotationTest) {
        Ellipsoid e(10.0, 5.0, 1.0, Vector3(10.0, 0.0, 0.0), Rotation::around_x(PI/2.0), 0.8);

        VtkVisualization vis;
        vis.insert_ellipsoid(e);

        vis.visualize();
    }

    TEST(VtkVisualizerTest, RotationOnlyTest) {
        Ellipsoid e(10.0, 5.0, 1.0,
                    Vector3(0.0, 0.0, 0.0),
                    Rotation::around_x(3.8 * PI / 2.0),
                    0.8);

        VtkVisualization vis;
        vis.insert_ellipsoid(e);
        vis.visualize();
    }

}
