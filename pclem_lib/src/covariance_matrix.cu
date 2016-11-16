
#include <glog/logging.h>
#include <armadillo>
#include "covariance_matrix.h"

namespace pclem {
    CovarianceMatrix::CovarianceMatrix() : values {{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0}} {}

    CovarianceMatrix::CovarianceMatrix(const CovarianceMatrix& other) {
        values = other.values;
    }

    CovarianceMatrix::CovarianceMatrix(std::array<double,9> _values) {
        for(int i = 0; i < 9; i++) {
            values[i] = _values[i];
        }
    }

    CovarianceMatrix CovarianceMatrix::zeros() {
        std::array<double,9> zeros = {0.0};
        return CovarianceMatrix(zeros);
    }

    void CovarianceMatrix::operator+=(const CovarianceMatrix& rhs) {
        for(int i = 0; i < 9; i++) {
            values[i] += rhs.values[i];
        }
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
        VLOG(11) << "Inverting matrix...";

        arma::mat33 arma_cov_mat(values.data());
        arma::mat33 arma_inv_of_cov = arma::inv(arma_cov_mat);

        std::array<double,9> inverse;
        for(auto i = 0; i < 3; i++) {
            for(auto j = 0; j < 3; j++) {
                inverse[i*3 + j] = arma_inv_of_cov(i,j);
            }
        }

        VLOG(11) << "Done inverting matrix.";
        return inverse;
    }

    std::pair<Vector3, std::array<Vector3,3>> CovarianceMatrix::eigenvalues() const {
        VLOG(10) << "Extracting eigenvalues...";

        arma::mat33 arma_cov_mat(values.data());
        arma::cx_vec arma_eigvals;
        arma::cx_mat arma_eigvecs;

        arma::eig_gen(arma_eigvals, arma_eigvecs, arma_cov_mat);

        Vector3 eigvals;
        std::array<Vector3, 3> eigvecs;

        for(int i=0; i < 3; i++) {
            eigvals[i] = arma_eigvals[i].real();

            for(int j=0; j < 3; j++) {
                eigvecs[i][j] = arma_eigvecs(j,i).real();
            }

            if(std::abs(arma_eigvals[i].imag()) > 1e-300) {
                LOG(WARNING) << "Imaginary part was found during eigenvalue extraction";
            }
        }

        return std::make_pair(eigvals, eigvecs);
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
