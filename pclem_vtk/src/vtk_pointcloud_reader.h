
#include <string>

#include "pointcloud.h"

namespace pclem {
    class VtkPointCloudReader {
    public:
        static PointCloud read(std::string filename);
        static PointCloud read(std::string filename, long n_points);
    private:
    };
}
