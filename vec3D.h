#pragma once

#pragma once
#include <cmath>

class Vec3D {
public:
    double x { 0.0 };
    double y { 0.0 };
    double z { 0.0 };

    Vec3D() = default;
    Vec3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3D operator+(const Vec3D& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3D operator-(const Vec3D& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3D operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vec3D operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }

    Vec3D& operator+=(const Vec3D& other) { x += other.x; y += other.y; z += other.z; return *this; }
    Vec3D& operator-=(const Vec3D& other) { x -= other.x; y -= other.y; z += other.z; return *this; }
    Vec3D& operator*=(double scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
    Vec3D& operator/=(double scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

    bool operator==(const Vec3D& other) const {return x == other.x && y == other.y && z == other.z;}

    bool operator!=(const Vec3D& other) const {return !(*this == other);
    }

    double length() const { return std::sqrt(x*x + y*y + z*z); }
    double length_squared() const { return x*x + y*y + z*z; }

    Vec3D normalized() const {
        double len = length();
        if (len == 0) return {0,0,0};
        return {x / len, y / len, z / len};
    }

    double dot(const Vec3D& other) const { return x * other.x + y * other.y + z * other.z; }

    void rotete(double angle) {
        double sinA = sin(angle);
        double cosA = cos(angle);

        double temp_x = x;
        double temp_y = y;

        x = cosA * temp_x - sinA * temp_y;
        y = sinA * temp_x + cosA * temp_y;
    }

    // ---- Вспомогательные ----
    void zero() { x = 0; y = 0; z = 0; }
};
