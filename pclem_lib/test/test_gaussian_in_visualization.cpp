
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "point.h"
#include "ellipsoid.h"
#include "weighted_gaussian.h"

namespace pclem {
    class MockVisualization : public Visualization {
    public:
        MOCK_METHOD1(insert_point, void(const Point&));
        MOCK_METHOD1(insert_ellipsoid, void(const Ellipsoid&));
    };

    TEST(InsertInVisualizationTest, EasyGaussianTest) {
        
    }

}
