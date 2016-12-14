
#include <gtest/gtest.h>

#include "matrix33.h"
#include "gaussian_mixture.h"
#include "weighted_gaussian.h"

namespace pclem {
    TEST(AssignationOperatorTest, AssignationTest) {
        std::vector<WeightedGaussian> gaussians;
        gaussians.push_back(WeightedGaussian(Point(1.0, 0.0, 0.0), Matrix33(), 1.0));

        GaussianMixture a(gaussians);
        GaussianMixture b;

        b = a;

        EXPECT_EQ(1, b.n_gaussians());
        EXPECT_DOUBLE_EQ(b.get_gaussian(0).get_mu().x, 1.0);
    }
}
