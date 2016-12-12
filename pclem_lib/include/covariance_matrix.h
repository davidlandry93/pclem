#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

#include <iostream>
#include <fstream>
#include <array>

#include "vector3.h"
#include "matrix33.h"

namespace pclem {
    class CovarianceMatrix {
    public:
        CovarianceMatrix();
        CovarianceMatrix(const CovarianceMatrix& other);
        CovarianceMatrix(std::array<double,9> _values);
        static CovarianceMatrix zeros();
        static CovarianceMatrix identity();
        void operator+=(const CovarianceMatrix& rhs);


        double get(int i, int j) const;
        void set(int i, int j, double value);
        double det() const;
        std::pair< Vector3, Matrix33> svd_decomposition() const;
        std::array<double,9> as_array() const;
        std::array<double,9> inverse() const;
        friend std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& m);
        friend std::ofstream& operator<<(std::ofstream& ofs, const CovarianceMatrix& m);
    private:
        std::array<double,9> values;
    };
}

#endif
