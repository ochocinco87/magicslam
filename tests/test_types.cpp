/**
 * Unit tests for core types
 */

#include <magicslam/core/types.hpp>
#include <iostream>
#include <cmath>
#include <cassert>

using namespace MagicSLAM;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    test_##name(); \
    std::cout << "PASSED\n"; \
} while(0)

#define ASSERT_NEAR(a, b, eps) assert(std::abs((a) - (b)) < (eps))

TEST(vec3_operations) {
    Vec3 a{1, 2, 3};
    Vec3 b{4, 5, 6};

    Vec3 sum = a + b;
    assert(toFloat(sum.x) == 5);
    assert(toFloat(sum.y) == 7);
    assert(toFloat(sum.z) == 9);

    Vec3 diff = b - a;
    assert(toFloat(diff.x) == 3);
    assert(toFloat(diff.y) == 3);
    assert(toFloat(diff.z) == 3);

    float dot = a.dot(b);
    ASSERT_NEAR(dot, 32, 0.001f);  // 1*4 + 2*5 + 3*6 = 32

    Vec3 cross = a.cross(b);
    ASSERT_NEAR(toFloat(cross.x), -3, 0.001f);
    ASSERT_NEAR(toFloat(cross.y), 6, 0.001f);
    ASSERT_NEAR(toFloat(cross.z), -3, 0.001f);

    float norm = a.norm();
    ASSERT_NEAR(norm, std::sqrt(14.0f), 0.001f);
}

TEST(quaternion_identity) {
    Quaternion q;
    assert(q.w == 1);
    assert(q.x == 0);
    assert(q.y == 0);
    assert(q.z == 0);
}

TEST(quaternion_axis_angle) {
    Vec3 axis{0, 0, 1};  // Z-axis
    float angle = 3.14159265f / 2.0f;  // 90 degrees

    Quaternion q = Quaternion::fromAxisAngle(axis, angle);

    // Rotate (1, 0, 0) by 90 degrees around Z should give (0, 1, 0)
    Vec3 v{1, 0, 0};
    Vec3 rotated = q.rotate(v);

    ASSERT_NEAR(toFloat(rotated.x), 0, 0.01f);
    ASSERT_NEAR(toFloat(rotated.y), 1, 0.01f);
    ASSERT_NEAR(toFloat(rotated.z), 0, 0.01f);
}

TEST(quaternion_multiply) {
    // Two 90-degree rotations around Z = 180 degrees
    Quaternion q1 = Quaternion::fromAxisAngle(Vec3{0, 0, 1}, 3.14159265f / 2.0f);
    Quaternion q2 = q1 * q1;

    Vec3 v{1, 0, 0};
    Vec3 rotated = q2.rotate(v);

    ASSERT_NEAR(toFloat(rotated.x), -1, 0.01f);
    ASSERT_NEAR(toFloat(rotated.y), 0, 0.01f);
    ASSERT_NEAR(toFloat(rotated.z), 0, 0.01f);
}

TEST(quaternion_slerp) {
    Quaternion q1;  // Identity
    Quaternion q2 = Quaternion::fromAxisAngle(Vec3{0, 0, 1}, 3.14159265f);  // 180 degrees

    Quaternion mid = Quaternion::slerp(q1, q2, 0.5f);

    // Halfway should be 90 degrees
    Vec3 v{1, 0, 0};
    Vec3 rotated = mid.rotate(v);

    ASSERT_NEAR(toFloat(rotated.x), 0, 0.1f);
    ASSERT_NEAR(toFloat(rotated.y), 1, 0.1f);
}

TEST(pose_transform) {
    Pose pose;
    pose.position = Vec3{1, 2, 3};
    pose.orientation = Quaternion::fromAxisAngle(Vec3{0, 0, 1}, 3.14159265f / 2.0f);

    Vec3 localPoint{1, 0, 0};
    Vec3 worldPoint = pose.transformPoint(localPoint);

    // Rotated 90 degrees + translated
    ASSERT_NEAR(toFloat(worldPoint.x), 1, 0.01f);
    ASSERT_NEAR(toFloat(worldPoint.y), 3, 0.01f);
    ASSERT_NEAR(toFloat(worldPoint.z), 3, 0.01f);

    // Inverse transform should give back local point
    Vec3 backToLocal = pose.inverseTransformPoint(worldPoint);
    ASSERT_NEAR(toFloat(backToLocal.x), 1, 0.01f);
    ASSERT_NEAR(toFloat(backToLocal.y), 0, 0.01f);
    ASSERT_NEAR(toFloat(backToLocal.z), 0, 0.01f);
}

TEST(pose_inverse) {
    Pose pose;
    pose.position = Vec3{1, 2, 3};
    pose.orientation = Quaternion::fromAxisAngle(Vec3{0, 1, 0}, 0.5f);

    Pose inv = pose.inverse();
    Pose identity = pose * inv;

    // Should be approximately identity
    ASSERT_NEAR(toFloat(identity.position.x), 0, 0.01f);
    ASSERT_NEAR(toFloat(identity.position.y), 0, 0.01f);
    ASSERT_NEAR(toFloat(identity.position.z), 0, 0.01f);
    ASSERT_NEAR(identity.orientation.w, 1, 0.01f);
}

TEST(camera_project_unproject) {
    CameraIntrinsics cam;
    cam.fx = 500;
    cam.fy = 500;
    cam.cx = 320;
    cam.cy = 240;
    cam.k1 = cam.k2 = cam.p1 = cam.p2 = cam.k3 = 0;
    cam.width = 640;
    cam.height = 480;

    Vec3 point{0.1f, 0.1f, 1.0f};  // Point at 1m depth
    Point2D projected = cam.project(point);

    // Should be offset from principal point
    ASSERT_NEAR(projected.x, 370, 1);
    ASSERT_NEAR(projected.y, 290, 1);

    // Unproject should give normalized ray
    Vec3 ray = cam.unproject(projected);
    Vec3 normalized = point.normalized();

    ASSERT_NEAR(toFloat(ray.x), toFloat(normalized.x), 0.01f);
    ASSERT_NEAR(toFloat(ray.y), toFloat(normalized.y), 0.01f);
    ASSERT_NEAR(toFloat(ray.z), toFloat(normalized.z), 0.01f);
}

TEST(mat3_operations) {
    Mat3 m = Mat3::identity();
    Vec3 v{1, 2, 3};
    Vec3 result = m * v;

    assert(toFloat(result.x) == 1);
    assert(toFloat(result.y) == 2);
    assert(toFloat(result.z) == 3);

    // Test rotation matrix from quaternion
    Quaternion q = Quaternion::fromAxisAngle(Vec3{0, 0, 1}, 3.14159265f / 2.0f);
    Mat3 R = Mat3::fromQuaternion(q);

    Vec3 v2{1, 0, 0};
    Vec3 rotated = R * v2;

    ASSERT_NEAR(toFloat(rotated.x), 0, 0.01f);
    ASSERT_NEAR(toFloat(rotated.y), 1, 0.01f);
}

int main() {
    std::cout << "Running core types tests...\n\n";

    RUN_TEST(vec3_operations);
    RUN_TEST(quaternion_identity);
    RUN_TEST(quaternion_axis_angle);
    RUN_TEST(quaternion_multiply);
    RUN_TEST(quaternion_slerp);
    RUN_TEST(pose_transform);
    RUN_TEST(pose_inverse);
    RUN_TEST(camera_project_unproject);
    RUN_TEST(mat3_operations);

    std::cout << "\nAll tests passed!\n";
    return 0;
}
