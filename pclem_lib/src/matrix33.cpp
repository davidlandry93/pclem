
#include "matrix33.h"

namespace pclem {
    Matrix33::Matrix33() : values{} {}

    // The values are given row-major.
    Matrix33::Matrix33(const std::array<double,9>& values) : values(values) {}

    double Matrix33::get_element(const int& i, const int& j) const {
        return values[i*3 + j];
    }

    std::ostream& operator<<(std::ostream& os, const Matrix33& v) {
        os << "[";
        for(int i=0; i < 8; i++) {
            os << v.values[i] << ",";
        }
        os << v.values[8] << "]";
    }
}
