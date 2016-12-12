
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
        arma::mat33 arma_inv_of_cov = arma::pinv(arma_cov_mat + arma::eye(3,3));

        VLOG(11) << "PseudoInverse error " << arma::norm(arma_cov_mat*arma_inv_of_cov - arma::eye(3,3));

        std::array<double,9> inverse;
        for(auto i = 0; i < 3; i++) {
            for(auto j = 0; j < 3; j++) {
                inverse[i*3 + j] = arma_inv_of_cov(i,j);
            }
        }

        VLOG(11) << "Done inverting matrix.";
        return inverse;
    }

    std::pair<Vector3, Matrix33> CovarianceMatrix::eigen_decomposition() const {
        VLOG(10) << "Extracting eigenvalues...";

        arma::mat33 arma_cov_mat(values.data());
        arma::vec arma_eigvals;
        arma::mat arma_eigvecs;

        if(!arma::eig_sym(arma_eigvals, arma_eigvecs, arma_cov_mat)) {
            std::cout << "DECOMPOTISION FAILDE";
            LOG(WARNING) << "Eigenvalues decomposition failed.";
        }

        Vector3 eigvals;
        std::array<double,9> eigvecs_values;
        for(int i=0; i < 3; i++) {
            eigvals[i] = arma_eigvals[i];
            std::cout << "Eigval " << eigvals[i] << std::endl;

            for(int j=0; j < 3; j++) {
                std::cout << arma_eigvecs(i,j);
                eigvecs_values[i*3 + j] = arma_eigvecs(i,j);
            }
        }

        return std::make_pair(eigvals, Matrix33(eigvecs_values));
    }

    std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& m) {
        os << "[";
        for(auto value : m.as_array()) {
            os << value << ",";
        }
        os << "]";

        return os;
    }

    std::ofstream& operator<<(std::ofstream& ofs, const CovarianceMatrix& m) {
        std::array<double,9> array_of_matrix = m.as_array();
        for(int i=0; i < 8; i++) {
            ofs << array_of_matrix[i] << ",";
        }
        ofs << array_of_matrix[8];

        return ofs;
    }

    CovarianceMatrix CovarianceMatrix::identity() {
        return CovarianceMatrix({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    }
}
