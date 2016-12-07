#ifndef FILE_EXPORTER_H
#define FILE_EXPORTER_H

#include <fstream>
#include <string>

#include "visualization.h"

namespace pclem {
    class FileExporter : public Visualization {
    public:
        FileExporter(const std::string& path_to_file);
        ~FileExporter();
        void open_file();
        void insert_point(const Point& point);
        void insert_ellipsoid(const Ellipsoid& ellipsoid);
        void close();

    private:
        const std::string CSV_HEADER;

        std::string filepath;
        std::ofstream filestream;
    };
}

#endif
