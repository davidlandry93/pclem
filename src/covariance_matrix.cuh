#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

#include <iostream>
#include <array>

#include "point.cuh"

namespace pclem {
    class CovarianceMatrix {
    public:
        __host__ __device__
        CovarianceMatrix() : values {{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0}} {}

        __host__ __device__
        CovarianceMatrix(const CovarianceMatrix& other) {
            values = other.values;
        }

        __host__ __device__
        Point operator*(const Point& rhs) {
            return Point(
                values[0]*rhs.x + values[1]*rhs.y + values[2]*rhs.z,
                values[3]*rhs.x + values[4]*rhs.y + values[5]*rhs.z,
                values[6]*rhs.x + values[7]*rhs.y + values[8]*rhs.z);
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
