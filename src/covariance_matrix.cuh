#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

#include <iostream>
#include <array>

#include "device_point.cuh"
#include "raw_covariance_matrix.h"

namespace pclem {
    class CovarianceMatrix {
    public:
        __host__ __device__
        CovarianceMatrix() : values {{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0}} {}

        __host__ __device__
        CovarianceMatrix(const CovarianceMatrix& other) {
            values = other.values;
        }

        CovarianceMatrix(double* _values) {
            for(int i = 0; i < 9; i++) {
                values[i] = _values[i];
            }
        }

        CovarianceMatrix(RawCovarianceMatrix& m);

        static CovarianceMatrix zeros() {
            double zeros[9] = {0.0};
            return CovarianceMatrix(zeros);
        }

        __host__ __device__
        DevicePoint operator*(const DevicePoint& rhs) {
            return DevicePoint(
                values[0]*rhs.x + values[1]*rhs.y + values[2]*rhs.z,
                values[3]*rhs.x + values[4]*rhs.y + values[5]*rhs.z,
                values[6]*rhs.x + values[7]*rhs.y + values[8]*rhs.z);
        }

        void operator+=(const CovarianceMatrix& rhs) {
            for(int i = 0; i < 9; i++) {
                values[i] += rhs.values[i];
            }
        }

        double get(int i, int j) const;
        void set(int i, int j, double value);
        double det() const;
        std::array<double,9> as_array() const;
        std::array<double,9> inverse() const;
        friend std::ostream& operator<<(std::ostream& os, const CovarianceMatrix& m);
    private:
        std::array<double,9> values;
    };
}

#endif
