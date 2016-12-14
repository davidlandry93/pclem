
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

        ASSERT_EQ(1, b.n_gaussians());
    }
}
