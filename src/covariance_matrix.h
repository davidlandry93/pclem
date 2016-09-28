#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

namespace pclem {
    class CovarianceMatrix {
    public:
        CovarianceMatrix();
        double get(int i, int j);
        void set(int i, int j, double value);
    private:
        double values[9];
    };
}

#endif
