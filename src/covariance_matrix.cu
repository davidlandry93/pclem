
#include <glog/logging.h>
#include <armadillo>
#include "covariance_matrix.cuh"

namespace pclem {
    CovarianceMatrix::CovarianceMatrix(RawCovarianceMatrix& m) {
        values[0] = m.v00;
        values[1] = m.v01;
        values[2] = m.v02;
        values[3] = m.v10;
        values[4] = m.v11;
        values[5] = m.v12;
        values[6] = m.v20;
        values[7] = m.v21;
        values[8] = m.v22;
    }

    double CovarianceMatrix::get(int i, int j) const {
        return values[i*3 + j];
    }

    void CovarianceMatrix::set(int i, int j, double new_value) {
        values[i*3 + j] = new_value;
    }

    double CovarianceMatrix::det() const {
        return get(0,0) * (get(1,1) * get(2,2) - get(2, 1) * get(1,2)) -
            get(0,1) * (get(1,0) * get(2,2) - get(1,2) * get(2,0)) +
            get(0,2) * (get(1,0) * get(2,1) + get(2,0) * get(1,1));
    }

    std::array<double,9> CovarianceMatrix::as_array() const {
        auto m = values;
        return m;
    }

    std::array<double,9> CovarianceMatrix::inverse() const {
        VLOG(10) << "Inverting matrix...";

        arma::mat33 arma_cov_mat(values.data());

        std::cout << "Matrix to invert: ";
        for(int i = 0; i < 9; i++) {
            std::cout << values[i] << " ";
        }
        std::cout <<std::endl;

        arma::mat33 arma_inv_of_cov = arma::inv(arma_cov_mat);

        std::array<double,9> inverse;
        for(auto i = 0; i < 3; i++) {
            for(auto j = 0; j < 3; j++) {
                inverse[i*3 + j] = arma_inv_of_cov(i,j);
            }
        }

        VLOG(10) << "Done inverting matrix.";
        return inverse;
    }

    std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& m) {
        os << "[";
        for(auto value : m.as_array()) {
            os << value << ",";
        }
        os << "]";

        return os;
    }
}
