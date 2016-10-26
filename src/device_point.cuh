#ifndef PCLEM_DEVICE_POINT_H
#define PCLEM_DEVICE_POINT_H

#include <iostream>

#include "point.h"

namespace pclem {
    class DevicePoint {
    public:
        double x,y,z;

        __host__ __device__
        DevicePoint() : x(0), y(0), z(0) {}

        __host__ __device__
        DevicePoint(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

        __host__ __device__
        DevicePoint(const DevicePoint& other) : x(other.x), y(other.y), z(other.z) {}

        __host__ __device__
        DevicePoint(const Point& other) : x(other.x), y(other.y), z(other.z) {}

        __host__ __device__
        DevicePoint operator-(const DevicePoint& other) {
            return DevicePoint(x - other.x, y - other.y, z - other.z);
        }

        __host__ __device__
        DevicePoint operator+(const DevicePoint& other) {
            return DevicePoint(x + other.x, y + other.y, z + other.z);
        }

        friend std::ostream& operator<<(std::ostream& os, const DevicePoint& p) {
            os << "(" << p.x << ","
               << p.y << ","
               << p.z << ")";
            return os;
        }

        Point to_host() const {
            return Point(x,y,z);
        }
    };
}

#endif
