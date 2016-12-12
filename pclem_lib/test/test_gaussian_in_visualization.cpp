
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "point.h"
#include "ellipsoid.h"
#include "weighted_gaussian.h"

using ::testing::_;
using ::testing::Field;
using ::testing::DoubleEq;
using ::testing::Eq;

namespace pclem {
    class MockVisualization : public Visualization {
    public:
        MOCK_METHOD1(insert_point, void(const Point&));
        MOCK_METHOD1(insert_ellipsoid, void(const Ellipsoid&));
    };

    TEST(InsertInVisualizationTest, EasyGaussianTest) {
        MockVisualization vis;

        Point mu(0.0, 0.0, 0.0);
        WeightedGaussian g(mu, CovarianceMatrix({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}), 1.0);

        Ellipsoid expected(1.0, 1.0, 1.0, Vector3(0.0, 0.0, 0.0), Matrix33({1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0}), 0.9);

        EXPECT_CALL(vis, insert_ellipsoid(_)).Times(1);

        g.insert_into_visualization(vis);
    }

}
