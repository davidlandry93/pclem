#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

namespace pclem {
    class CovarianceMatrix {
    public:
        CovarianceMatrix();

        __host__ __device__
        CovarianceMatrix(const CovarianceMatrix& other);

        __host__ __device__
        double get(int i, int j) const;
        void set(int i, int j, double value);

        __host__ __device__
        double det() const;
    private:
        double values[9];
    };
}

#endif
