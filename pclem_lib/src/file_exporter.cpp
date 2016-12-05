
#include <iostream>

#include "file_exporter.h"

namespace pclem  {
    const std::string CSV_HEADER = "a,b,c,x,y,z,rot11,rot12,rot13,rot21,rot22,rot23,rot31,rot32,rot33,opacity";

    FileExporter::FileExporter(const std::string& path_to_file) :
        filepath(path_to_file) {}

    FileExporter::~FileExporter() {
        filestream.close();
    }

    void FileExporter::open_file() {
        filestream.open(filepath, std::ios::out);
        filestream << CSV_HEADER << std::endl;
    }

    void FileExporter::insert_point(const Point& point) {
        filestream << point;
    }

    void FileExporter::insert_ellipsoid(const Ellipsoid& ellipsoid) {
        filestream << ellipsoid;
    }
}
