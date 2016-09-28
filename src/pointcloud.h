
#include <memory>
#include <vector>
#include <vtkPolyData.h>

#include "boundingbox.h"

namespace pclem {

    class PointCloud {
    public:
        static PointCloud from_vtk(vtkPolyData* vtkData);
    private:
        PointCloud(std::vector<double> data, int n_of_points);
        std::vector<double> data;
        int n_of_points;
        BoundingBox boundingBox;
    };

}
