#ifndef ROTATION_H
#define ROTATION_H

#include "matrix33.h"

namespace pclem {
    class Rotation {
    public:
        Rotation();
        Rotation(const Matrix33& matrix);
        Matrix33 as_matrix() const;
        std::pair<Vector3,double> as_axis_angle() const;
        bool operator==(const Rotation& other) const;

        static Rotation around_x(double rads);

        friend std::ostream& operator<<(std::ostream& os, const Rotation r);

    private:
        Matrix33 matrix;
    };
}

#endif
