#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <cmath>
#include <memory>
#include <chrono>

namespace MagicSLAM {

// Use fixed-point for AR1 optimization when needed
#ifdef USE_FIXED_POINT
using Scalar = int32_t;
constexpr int FIXED_POINT_BITS = 16;
inline float toFloat(Scalar v) { return static_cast<float>(v) / (1 << FIXED_POINT_BITS); }
inline Scalar toFixed(float v) { return static_cast<Scalar>(v * (1 << FIXED_POINT_BITS)); }
#else
using Scalar = float;
inline float toFloat(Scalar v) { return v; }
inline Scalar toFixed(float v) { return v; }
#endif

using Timestamp = uint64_t;  // Nanoseconds since epoch

// 3D Vector
struct Vec3 {
    Scalar x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(Scalar x_, Scalar y_, Scalar z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(Scalar s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    Scalar dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    Scalar norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        Scalar n = norm();
        return n > 1e-8f ? Vec3{x / n, y / n, z / n} : Vec3{};
    }
};

// Quaternion for rotation (w, x, y, z)
struct Quaternion {
    Scalar w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(Scalar w_, Scalar x_, Scalar y_, Scalar z_) : w(w_), x(x_), y(y_), z(z_) {}

    // Create from axis-angle
    static Quaternion fromAxisAngle(const Vec3& axis, Scalar angle) {
        Scalar halfAngle = angle * 0.5f;
        Scalar s = std::sin(halfAngle);
        Vec3 n = axis.normalized();
        return {std::cos(halfAngle), n.x * s, n.y * s, n.z * s};
    }

    // Create from rotation vector (axis * angle)
    static Quaternion fromRotationVector(const Vec3& rv) {
        Scalar angle = rv.norm();
        if (angle < 1e-8f) {
            return Quaternion{1.0f, rv.x * 0.5f, rv.y * 0.5f, rv.z * 0.5f}.normalized();
        }
        return fromAxisAngle(rv * (1.0f / angle), angle);
    }

    Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    Vec3 rotate(const Vec3& v) const {
        // Optimized quaternion-vector rotation
        Vec3 qv{x, y, z};
        Vec3 uv = qv.cross(v);
        Vec3 uuv = qv.cross(uv);
        return v + (uv * w + uuv) * 2.0f;
    }

    Quaternion conjugate() const { return {w, -x, -y, -z}; }
    Quaternion inverse() const { return conjugate(); }  // Assuming unit quaternion

    Scalar norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }
    Quaternion normalized() const {
        Scalar n = norm();
        return {w / n, x / n, y / n, z / n};
    }

    // Convert to rotation vector
    Vec3 toRotationVector() const {
        Scalar sinHalfAngle = std::sqrt(x * x + y * y + z * z);
        if (sinHalfAngle < 1e-8f) {
            return {x * 2.0f, y * 2.0f, z * 2.0f};
        }
        Scalar angle = 2.0f * std::atan2(sinHalfAngle, w);
        Scalar scale = angle / sinHalfAngle;
        return {x * scale, y * scale, z * scale};
    }

    // SLERP interpolation
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, Scalar t) {
        Scalar dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        Quaternion b2 = b;
        if (dot < 0) {
            dot = -dot;
            b2 = {-b.w, -b.x, -b.y, -b.z};
        }

        if (dot > 0.9995f) {
            // Linear interpolation for very close quaternions
            return Quaternion{
                a.w + t * (b2.w - a.w),
                a.x + t * (b2.x - a.x),
                a.y + t * (b2.y - a.y),
                a.z + t * (b2.z - a.z)
            }.normalized();
        }

        Scalar theta0 = std::acos(dot);
        Scalar theta = theta0 * t;
        Scalar sinTheta = std::sin(theta);
        Scalar sinTheta0 = std::sin(theta0);

        Scalar s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
        Scalar s1 = sinTheta / sinTheta0;

        return {
            s0 * a.w + s1 * b2.w,
            s0 * a.x + s1 * b2.x,
            s0 * a.y + s1 * b2.y,
            s0 * a.z + s1 * b2.z
        };
    }
};

// 3x3 Matrix (row-major)
struct Mat3 {
    std::array<Scalar, 9> data;

    Mat3() : data{1, 0, 0, 0, 1, 0, 0, 0, 1} {}

    Scalar& operator()(int r, int c) { return data[r * 3 + c]; }
    Scalar operator()(int r, int c) const { return data[r * 3 + c]; }

    static Mat3 identity() { return Mat3{}; }

    static Mat3 fromQuaternion(const Quaternion& q) {
        Mat3 m;
        Scalar xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        Scalar xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        Scalar wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        m(0, 0) = 1 - 2 * (yy + zz);
        m(0, 1) = 2 * (xy - wz);
        m(0, 2) = 2 * (xz + wy);
        m(1, 0) = 2 * (xy + wz);
        m(1, 1) = 1 - 2 * (xx + zz);
        m(1, 2) = 2 * (yz - wx);
        m(2, 0) = 2 * (xz - wy);
        m(2, 1) = 2 * (yz + wx);
        m(2, 2) = 1 - 2 * (xx + yy);
        return m;
    }

    Vec3 operator*(const Vec3& v) const {
        return {
            data[0] * v.x + data[1] * v.y + data[2] * v.z,
            data[3] * v.x + data[4] * v.y + data[5] * v.z,
            data[6] * v.x + data[7] * v.y + data[8] * v.z
        };
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r(i, j) = 0;
                for (int k = 0; k < 3; ++k) {
                    r(i, j) += (*this)(i, k) * o(k, j);
                }
            }
        }
        return r;
    }

    Mat3 transpose() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r(i, j) = (*this)(j, i);
        return r;
    }
};

// SE(3) Pose - Position and Orientation
struct Pose {
    Vec3 position;
    Quaternion orientation;

    Pose() = default;
    Pose(const Vec3& pos, const Quaternion& ori) : position(pos), orientation(ori) {}

    // Transform a point from local to world coordinates
    Vec3 transformPoint(const Vec3& p) const {
        return orientation.rotate(p) + position;
    }

    // Transform a point from world to local coordinates
    Vec3 inverseTransformPoint(const Vec3& p) const {
        return orientation.inverse().rotate(p - position);
    }

    // Compose two poses (this * other)
    Pose operator*(const Pose& other) const {
        return {
            transformPoint(other.position),
            orientation * other.orientation
        };
    }

    // Inverse pose
    Pose inverse() const {
        Quaternion invOri = orientation.inverse();
        return {invOri.rotate(position * -1.0f), invOri};
    }

    // Interpolate between poses
    static Pose interpolate(const Pose& a, const Pose& b, Scalar t) {
        return {
            Vec3{
                a.position.x + t * (b.position.x - a.position.x),
                a.position.y + t * (b.position.y - a.position.y),
                a.position.z + t * (b.position.z - a.position.z)
            },
            Quaternion::slerp(a.orientation, b.orientation, t)
        };
    }
};

// 2D point (image coordinates)
struct Point2D {
    float x, y;
    Point2D() : x(0), y(0) {}
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

// Camera intrinsics
struct CameraIntrinsics {
    float fx, fy;       // Focal lengths
    float cx, cy;       // Principal point
    float k1, k2, p1, p2, k3;  // Distortion coefficients
    int width, height;

    // Project 3D point to 2D
    Point2D project(const Vec3& p) const {
        if (std::abs(p.z) < 1e-6f) return {-1, -1};
        float xn = toFloat(p.x) / toFloat(p.z);
        float yn = toFloat(p.y) / toFloat(p.z);

        // Apply distortion
        float r2 = xn * xn + yn * yn;
        float r4 = r2 * r2;
        float r6 = r4 * r2;
        float radial = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
        float xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
        float yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;

        return {fx * xd + cx, fy * yd + cy};
    }

    // Unproject 2D point to normalized ray
    Vec3 unproject(const Point2D& p) const {
        float xn = (p.x - cx) / fx;
        float yn = (p.y - cy) / fy;
        // Iterative undistortion (simplified for speed)
        float x = xn, y = yn;
        for (int i = 0; i < 5; ++i) {
            float r2 = x * x + y * y;
            float radial = 1.0f + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            x = xn / radial;
            y = yn / radial;
        }
        return Vec3{toFixed(x), toFixed(y), toFixed(1.0f)}.normalized();
    }

    bool isInImage(const Point2D& p, float margin = 0) const {
        return p.x >= margin && p.x < width - margin &&
               p.y >= margin && p.y < height - margin;
    }
};

// IMU measurement
struct ImuMeasurement {
    Timestamp timestamp;
    Vec3 acceleration;  // m/s^2
    Vec3 angularVelocity;  // rad/s

    ImuMeasurement() : timestamp(0) {}
    ImuMeasurement(Timestamp ts, const Vec3& acc, const Vec3& gyro)
        : timestamp(ts), acceleration(acc), angularVelocity(gyro) {}
};

// IMU biases
struct ImuBias {
    Vec3 accelerometer;
    Vec3 gyroscope;

    ImuBias() = default;
    ImuBias(const Vec3& acc, const Vec3& gyro) : accelerometer(acc), gyroscope(gyro) {}
};

// Motion state for prediction
struct MotionState {
    Pose pose;
    Vec3 velocity;
    Vec3 angularVelocity;
    Timestamp timestamp;

    MotionState() : timestamp(0) {}
};

}  // namespace MagicSLAM
