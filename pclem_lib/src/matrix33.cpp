
#include "pclem_math.h"

#include "matrix33.h"

namespace pclem {
    Matrix33::Matrix33() : values{} {}

    // The values are given row-major.
    Matrix33::Matrix33(const std::array<double,9>& values) : values(values) {}

    double Matrix33::get_element(const int& i, const int& j) const {
        return values[i*3 + j];
    }

    Matrix33 Matrix33::identity() {
        return Matrix33({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    }

    bool Matrix33::operator==(const Matrix33& other) const {
        for(int i=0; i < 9; i++) {
            if(!approximatelyEqual(values[i], other.values[i], 1e-10)) {
                return false;
            }
        }
        return true;
    }

    std::ostream& operator<<(std::ostream& os, const Matrix33& v) {
        for(int i=0; i < 8; i++) {
            os << v.values[i] << ",";
        }
        os << v.values[8];
    }

    std::ofstream& operator<<(std::ofstream& ofs, const Matrix33& v) {
        for(int i=0; i < 8; i++) {
            ofs << v.values[i] << ",";
        }
        ofs << v.values[8];

        return ofs;
    }
}
