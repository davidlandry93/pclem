
#include "covariance_matrix.h"

namespace pclem {
    CovarianceMatrix::CovarianceMatrix() {
        for(auto value : values) {
            value = 0;
        }
    }

    double CovarianceMatrix::get(int i, int j) const {
        return values[i*3 + j];
    }

    void CovarianceMatrix::set(int i, int j, double value) {
        values[i*3 + j] = value;
    }

    double CovarianceMatrix::det() const {
        return get(0,0) * (get(1,1) * get(2,2) - get(2, 1) * get(1,2)) -
            get(0,1) * (get(1,0) * get(2,2) - get(1,2) * get(2,0)) +
            get(0,2) * (get(1,0) * get(2,1) + get(2,0) * get(1,1));
    }
}
