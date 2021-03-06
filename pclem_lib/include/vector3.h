#ifndef VECTOR3_H
#define VECTOR3_H

#include <iostream>
#include <fstream>

namespace pclem {

    class Vector3 {
    public:
        double x,y,z;

        Vector3();
        Vector3(double x, double y, double z);
        Vector3(const Vector3& other);
        void normalize();
        double length();
        double& operator[](int index);
        void operator=(const Vector3& other);
        bool operator==(const Vector3& other) const;
        double dot(const Vector3& other) const;

        friend std::ostream& operator<<(std::ostream& os, const Vector3& v);
        friend std::ofstream& operator<<(std::ofstream& ofs, const Vector3& v);
    };

}

#endif
