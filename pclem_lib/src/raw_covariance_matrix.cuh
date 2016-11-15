#ifndef RAW_COVARIANCE_MATRIX_H
#define RAW_COVARIANCE_MATRIX_H

#include "covariance_matrix.h"

namespace pclem{


struct RawCovarianceMatrix {
public:
    double v00, v01, v02,
        v10, v11, v12,
        v20, v21, v22;

    static RawCovarianceMatrix zeros() {
        RawCovarianceMatrix r;

        r.v00 = 0.0;
        r.v01 = 0.0;
        r.v02 = 0.0;
        r.v10 = 0.0;
        r.v11 = 0.0;
        r.v12 = 0.0;
        r.v20 = 0.0;
        r.v21 = 0.0;
        r.v22 = 0.0;

        return r;
    }

    __host__ __device__
    void operator+=(const RawCovarianceMatrix& rhs) {
        v00 += rhs.v00;
        v01 += rhs.v01;
        v02 += rhs.v02;
        v10 += rhs.v10;
        v11 += rhs.v11;
        v12 += rhs.v12;
        v20 += rhs.v20;
        v21 += rhs.v21;
        v22 += rhs.v22;
    }

    __host__ __device__
    RawCovarianceMatrix operator+(const RawCovarianceMatrix& rhs) {
        RawCovarianceMatrix r;

        r.v00 = v00 + rhs.v00;
        r.v01 = v01 + rhs.v01;
        r.v02 = v02 + rhs.v02;
        r.v10 = v10 + rhs.v10;
        r.v11 = v11 + rhs.v11;
        r.v12 = v12 + rhs.v12;
        r.v20 = v20 + rhs.v20;
        r.v21 = v21 + rhs.v21;
        r.v22 = v22 + rhs.v22;

        return r;
    }

    __host__ __device__
    RawCovarianceMatrix operator*(const double& scalar) {
        RawCovarianceMatrix r;

        r.v00 = v00 * scalar;
        r.v01 = v01 * scalar;
        r.v02 = v02 * scalar;
        r.v10 = v10 * scalar;
        r.v11 = v11 * scalar;
        r.v12 = v12 * scalar;
        r.v20 = v20 * scalar;
        r.v21 = v21 * scalar;
        r.v22 = v22 * scalar;

        return r;
    }

    __host__ __device__
    RawCovarianceMatrix operator/(const double& scalar) {
        RawCovarianceMatrix r;

        r.v00 = v00 / scalar;
        r.v01 = v01 / scalar;
        r.v02 = v02 / scalar;
        r.v10 = v10 / scalar;
        r.v11 = v11 / scalar;
        r.v12 = v12 / scalar;
        r.v20 = v20 / scalar;
        r.v21 = v21 / scalar;
        r.v22 = v22 / scalar;

        return r;
    }

    __host__ __device__
    RawCovarianceMatrix operator-(const RawCovarianceMatrix& rhs) {
        RawCovarianceMatrix r;

        r.v00 = v00 - rhs.v00;
        r.v01 = v01 - rhs.v01;
        r.v02 = v02 - rhs.v02;
        r.v10 = v10 - rhs.v10;
        r.v11 = v11 - rhs.v11;
        r.v12 = v12 - rhs.v12;
        r.v20 = v20 - rhs.v20;
        r.v21 = v21 - rhs.v21;
        r.v22 = v22 - rhs.v22;

        return r;
    }

    CovarianceMatrix to_host() const {
        std::array<double,9> values = {{v00, v01, v02, v10, v11, v12, v20, v21, v22}};
        return CovarianceMatrix(values);
    }
};

}

#endif
