#ifndef VISUALIZABLE_POINTCLOUD_H
#define VISUALIZABLE_POINTCLOUD_H

#include <pclem/pointcloud.h>

#include "visualization.h"

namespace pclem {
    class VisualizablePointCloud : public PointCloud {
    public:
        void insert_in_visualization(Visualization& vis) const;
    };
}

#endif
