#ifndef PARULA_LOOKUP_TABLE_H
#define PARULA_LOOKUP_TABLE_H

#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>

namespace pclem {
    class ParulaLookupTable {
    public:
        ParulaLookupTable(double min, double max);
        void get_color(double value, double* color) const;
    private:
        static const int N_COLORS = 64;
        static const double COLORS[][3];

        vtkSmartPointer<vtkLookupTable> vtk_table;
    };
}

#endif
