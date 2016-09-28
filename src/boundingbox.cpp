
#include "boundingbox.h"

namespace pclem {
    BoundingBox::BoundingBox() :
        xmin(0.0), ymin(0.0), zmin(0.0),
        xmax(0.0), ymax(0.0), zmax(0.0) {
    }

    BoundingBox::BoundingBox(double xmin, double ymin, double zmin,
                             double xmax, double ymax, double zmax) :
        xmin(xmin), ymin(ymin), zmin(zmin),
        xmax(xmax), ymax(ymax), zmax(zmax) { }

    void BoundingBox::setMin(double pxmin, double pymin, double pzmin) {
        xmin = pxmin;
        ymin = pymin;
        zmin = pzmin;
    }

    void BoundingBox::setMax(double pxmax, double pymax, double pzmax) {
        xmax = pxmax;
        ymax = pymax;
        zmax = pzmax;
    }
}
