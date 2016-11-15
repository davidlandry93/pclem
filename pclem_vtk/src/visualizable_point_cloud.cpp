
#include "visualizable_point_cloud.h"

namespace pclem {
    void VisualizablePointCloud::insert_in_visualization(Visualization& vis) const {
        std::vector<Point> points = copy_of_points();

        for(Point point : points) {
            vis.insert_point(point);
        }
    }
}
