#ifndef PCLEM_MATH_H
#define PCLEM_MATH_H

#include <cmath>
#include "pclem_math.h"

namespace pclem {
    bool approximatelyEqual(double a, double b, double epsilon) {
        return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
    }
}

#endif
