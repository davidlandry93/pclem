#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

#include <iostream>
#include <array>

namespace pclem {
    class CovarianceMatrix {
    public:
        CovarianceMatrix();
        CovarianceMatrix(const CovarianceMatrix& other);
        CovarianceMatrix(std::array<double,9> _values);
        static CovarianceMatrix zeros();
        void operator+=(const CovarianceMatrix& rhs);


        double get(int i, int j) const;
        void set(int i, int j, double value);
        double det() const;
        std::array<double,3> eigenvalues() const;
        std::array<double,9> as_array() const;
        std::array<double,9> inverse() const;
        friend std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& m);
    private:
        std::array<double,9> values;
    };
}

#endif
