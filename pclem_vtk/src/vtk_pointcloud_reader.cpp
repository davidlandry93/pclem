
#include <limits>

#include <vtkSmartPointer.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkPolyData.h>
#include <glog/logging.h>

#include <pclem/point.h>

#include "vtk_pointcloud_reader.h"

namespace pclem{

    VisualizablePointCloud VtkPointCloudReader::read(std::string filename) {
        return read(filename, std::numeric_limits<long>::max());
    }

    VisualizablePointCloud VtkPointCloudReader::read(std::string filename, long max_n_points) {
        VLOG(10) << "Building point cloud...";

        vtkSmartPointer<vtkGenericDataObjectReader> reader =
            vtkSmartPointer<vtkGenericDataObjectReader>::New();
        reader->SetFileName(filename.c_str());
        reader->Update();

        vtkPolyData* vtkData = reader->GetPolyDataOutput();

        vtkIdType npoints = vtkData->GetNumberOfPoints();
        vtkPoints* points = vtkData->GetPoints();

        auto stl_vec = std::vector<Point>();
        for(int i=0; i < npoints && i < max_n_points; i++) {
            double currentPoint[3];

            points->GetPoint(i, currentPoint);
            stl_vec.push_back(Point(currentPoint[0], currentPoint[1], currentPoint[2]));
        }

        VLOG(10) << "Initializing a point cloud.";
        VisualizablePointCloud pcl;
        pcl.set_points(stl_vec);

        VLOG(10) << "Done.";
        return pcl;
    }

}
