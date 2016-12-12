
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
