
#include "glog/logging.h"
#include <armadillo>

#include "pclem_math.h"

#include "matrix33.h"

namespace pclem {
    Matrix33::Matrix33() : values{} {}

    // The values are given row-major.
    Matrix33::Matrix33(const std::array<double,9>& values) : values(values) {}

    double Matrix33::get_element(const int& i, const int& j) const {
        return values[i*3 + j];
    }

    Matrix33 Matrix33::identity() {
        return Matrix33({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    }

    bool Matrix33::operator==(const Matrix33& other) const {
        for(int i=0; i < 9; i++) {
            if(!approximatelyEqual(values[i], other.values[i], 1e-10)) {
                return false;
            }
        }
        return true;
    }

    std::pair<Vector3, Matrix33> Matrix33::eigen_decomposition() const {

        VLOG(10) << "Extracting eigenvalues...";

        arma::mat33 arma_cov_mat(values.data());
        arma::vec arma_eigvals;
        arma::mat arma_eigvecs;

        if(!arma::eig_sym(arma_eigvals, arma_eigvecs, arma_cov_mat)) {
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

    std::array<double,9> Matrix33::inverse() const {
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

    double Matrix33::det() const {
        return get(0,0) * (get(1,1) * get(2,2) - get(2, 1) * get(1,2)) -
            get(0,1) * (get(1,0) * get(2,2) - get(1,2) * get(2,0)) +
            get(0,2) * (get(1,0) * get(2,1) + get(2,0) * get(1,1));
    }

    std::ostream& operator<<(std::ostream& os, const Matrix33& v) {
        for(int i=0; i < 8; i++) {
            os << v.values[i] << ",";
        }
        os << v.values[8];
    }

    void Matrix33::set_element(const int& i, const int& j, const double value) {
        values[i*3 + j] = value;
    }

    std::ofstream& operator<<(std::ofstream& ofs, const Matrix33& v) {
        for(int i=0; i < 8; i++) {
            ofs << v.values[i] << ",";
        }
        ofs << v.values[8];

        return ofs;
    }
}
