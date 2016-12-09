#ifndef MATRIX_33_H
#define MATRIX_33_H

#include <iostream>
#include <fstream>
#include <array>

namespace pclem {
class Matrix33 {
public:
    Matrix33();
    Matrix33(const std::array<double,9>& values);
    double get_element(const int& i, const int& j) const;

    static Matrix33 identity();

    friend std::ostream& operator<<(std::ostream& os, const Matrix33& v);
    friend std::ofstream& operator<<(std::ofstream& os, const Matrix33& v);
private:
    std::array<double,9> values;

};
}

#endif
