
#include "gtest/gtest.h"
#include "covariance_matrix.h"
#include "matrix33.h"
#include "vector3.h"

namespace pclem {
    TEST(CovarianceMatrixTest, EigenValuesInOrder) {
        CovarianceMatrix c({1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0});

        auto decomposition = c.eigen_decomposition();
        auto eigvals = decomposition.first;

        EXPECT_LE(eigvals[0], eigvals[1]);
        EXPECT_LE(eigvals[1], eigvals[2]);
    }

    TEST(CovarianceMatrixTest, EivenVectorsInOrder) {
        CovarianceMatrix c({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        auto decomposition = c.eigen_decomposition();

        auto eigvecs = decompositions.second;

    }
}
