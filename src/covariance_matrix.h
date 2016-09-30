#ifndef COVARIANCE_MATRIX_H
#define COVARIANCE_MATRIX_H

namespace pclem {
    class CovarianceMatrix {
    public:
        CovarianceMatrix();
        double get(int i, int j) const;
        void set(int i, int j, double value);
        double det() const;
    private:
        double values[9];
    };
}

#endif
