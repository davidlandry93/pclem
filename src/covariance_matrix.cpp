
#include "covariance_matrix.h"

namespace pclem {
    CovarianceMatrix::CovarianceMatrix() {
        for(auto value : values) {
            value = 0;
        }
    }

    double CovarianceMatrix::get(int i, int j) {
        return values[i*3 + j];
    }

    void CovarianceMatrix::set(int i, int j, double value) {
        values[i*3 + j] = value;
    }
}
