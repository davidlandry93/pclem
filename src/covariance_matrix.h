#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

#include <iostream>
#include <array>

namespace pclem {
    class CovarianceMatrix {
    public:
        __host__ __device__
        CovarianceMatrix() : values {{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0}} {}

        __host__ __device__
        CovarianceMatrix(const CovarianceMatrix& other) {
            values = other.values;
        }

        double get(int i, int j) const;
        void set(int i, int j, double value);
        double det() const;
        std::array<double,9> as_array() const;
        friend std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& m);
    private:
        std::array<double,9> values;
    };
}

#endif
