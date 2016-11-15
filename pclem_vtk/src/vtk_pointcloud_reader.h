
#include <string>

#include "visualizable_point_cloud.h"

namespace pclem {
    class VtkPointCloudReader {
    public:
        static VisualizablePointCloud read(std::string filename);
        static VisualizablePointCloud read(std::string filename, long n_points);
    private:
    };
}
