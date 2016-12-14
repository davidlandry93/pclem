
#include <cmath>

#include "glog/logging.h"

#include "pclem_math.h"
#include "rotation.h"

namespace pclem {
    Rotation::Rotation() : matrix({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {}

    Rotation::Rotation(const Matrix33& rotation) : matrix(rotation) {}

    Matrix33 Rotation::as_matrix() const {
        return matrix;
    }

    std::pair<Vector3, double> Rotation::as_axis_angle() const {
        Vector3 axis;
        double angle;

        auto decomposition = matrix.eigen_decomposition();

        if(approximatelyEqual(decomposition.first.x, 1, 1e-20)) {
            axis = decomposition.second.get_column(0);
        } else if (approximatelyEqual(decomposition.first.y, 1, 1e-20)) {
            axis = decomposition.second.get_column(1);
        } else if (approximatelyEqual(decomposition.first.z, 1, 1e-20)) {
            axis = decomposition.second.get_column(2);
        } else {
            LOG(ERROR) << "Matrix is not a rotation matrix";

            std::cout << decomposition.first << std::endl;
            std::cout << decomposition.second << std::endl;
        }

        angle = acos((matrix.trace() - 1) / 2.0);

        return std::make_pair(axis,angle);
    }

    bool Rotation::operator==(const Rotation& other) const {
        return matrix == other.matrix;
    }

    Rotation Rotation::around_x(double rads) {
        std::array<double,9> values = {1.0, 0.0, 0.0, 0.0, cos(rads), -1*sin(rads), 0.0, sin(rads), cos(rads)};
        return Rotation(Matrix33(values));
    }

    std::ostream& operator<<(std::ostream& os, const Rotation r) {
        os << r.matrix;
        return os;
    }
}
