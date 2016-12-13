
#include <math.h>

#include "gtest/gtest.h"

#include "rotation.h"

namespace pclem {
    TEST(ConversionToAxisAngle, NullRotation) {
        Rotation r(Matrix33::identity());

        auto axis_angle = r.as_axis_angle();

        EXPECT_DOUBLE_EQ(axis_angle.second, 0.0);
    }

    TEST(ConversionToAxisAngle, RotationAroundX) {
        auto axis_angle = Rotation::around_x(M_PI).as_axis_angle();

        EXPECT_DOUBLE_EQ(axis_angle.second, M_PI);
        EXPECT_DOUBLE_EQ(axis_angle.first.x, 1.0);
    }
}
