/**
 * Unit tests for IMU integration
 */

#include <magicslam/imu/imu_integrator.hpp>
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

TEST(preintegration_stationary) {
    ImuPreintegration preint;
    ImuBias bias;

    // Simulate stationary IMU (only gravity)
    ImuMeasurement imu;
    imu.acceleration = Vec3{0, 0, 9.81f};  // Gravity pointing up in sensor frame
    imu.angularVelocity = Vec3{0, 0, 0};

    float dt = 0.005f;  // 5ms
    for (int i = 0; i < 200; ++i) {  // 1 second
        imu.timestamp = i * 5000000;
        preint.integrate(imu, bias, dt);
    }

    // After 1 second of stationary, delta position and velocity should be small
    // (gravity is removed in prediction step)
    ASSERT_NEAR(preint.getDeltaTime(), 1.0f, 0.01f);

    // Rotation should be identity
    const Quaternion& dq = preint.getDeltaRotation();
    ASSERT_NEAR(dq.w, 1.0f, 0.01f);
}

TEST(preintegration_rotation) {
    ImuPreintegration preint;
    ImuBias bias;

    // Simulate constant rotation around Z axis
    ImuMeasurement imu;
    imu.acceleration = Vec3{0, 0, 9.81f};
    imu.angularVelocity = Vec3{0, 0, 0.1f};  // 0.1 rad/s around Z

    float dt = 0.005f;
    for (int i = 0; i < 200; ++i) {
        imu.timestamp = i * 5000000;
        preint.integrate(imu, bias, dt);
    }

    // After 1 second, should have rotated 0.1 rad
    Vec3 rotVec = preint.getDeltaRotation().toRotationVector();
    ASSERT_NEAR(toFloat(rotVec.z), 0.1f, 0.01f);
}

TEST(integrator_buffer) {
    ImuIntegrator integrator;
    integrator.setGravity(Vec3{0, 0, -9.81f});

    ImuMeasurement imu;
    imu.acceleration = Vec3{0, 0, 9.81f};
    imu.angularVelocity = Vec3{0, 0, 0};

    // Add 100 measurements
    for (int i = 0; i < 100; ++i) {
        imu.timestamp = i * 5000000;
        integrator.addMeasurement(imu);
    }

    // Get latest should work
    ImuMeasurement latest;
    bool gotLatest = integrator.getLatest(latest);
    assert(gotLatest);
    assert(latest.timestamp == 99 * 5000000);
}

TEST(integrator_prediction) {
    ImuIntegrator integrator;
    integrator.setGravity(Vec3{0, 0, -9.81f});

    // Start at origin
    MotionState initial;
    initial.pose.position = Vec3{0, 0, 0};
    initial.velocity = Vec3{1, 0, 0};  // Moving 1 m/s in X
    initial.timestamp = 0;

    ImuMeasurement imu;
    imu.acceleration = Vec3{0, 0, 9.81f};  // Compensating gravity
    imu.angularVelocity = Vec3{0, 0, 0};

    // Add IMU measurements for 1 second
    for (int i = 0; i < 200; ++i) {
        imu.timestamp = i * 5000000;
        integrator.addMeasurement(imu);
    }

    // Predict forward
    MotionState predicted = integrator.predict(initial);

    // After 1 second moving at 1 m/s, should have moved ~1m in X
    // (with some gravity effect)
    ASSERT_NEAR(toFloat(predicted.pose.position.x), 1.0f, 0.5f);
}

TEST(bias_handling) {
    ImuPreintegration preint;

    // Simulated bias
    ImuBias bias;
    bias.accelerometer = Vec3{0.1f, 0.1f, 0.1f};
    bias.gyroscope = Vec3{0.01f, 0.01f, 0.01f};

    ImuMeasurement imu;
    imu.acceleration = Vec3{0.1f, 0.1f, 9.91f};  // Bias included
    imu.angularVelocity = Vec3{0.01f, 0.01f, 0.11f};  // Bias + 0.1 rad/s Z

    float dt = 0.005f;
    for (int i = 0; i < 200; ++i) {
        imu.timestamp = i * 5000000;
        preint.integrate(imu, bias, dt);
    }

    // With bias removed, should have rotated 0.1 rad around Z
    Vec3 rotVec = preint.getDeltaRotation().toRotationVector();
    ASSERT_NEAR(toFloat(rotVec.z), 0.1f, 0.02f);
}

TEST(predict_to_time) {
    ImuIntegrator integrator;
    integrator.setGravity(Vec3{0, 0, -9.81f});

    MotionState current;
    current.pose.position = Vec3{0, 0, 0};
    current.velocity = Vec3{1, 0, 0};
    current.angularVelocity = Vec3{0, 0, 0.1f};
    current.timestamp = 0;

    // Predict 100ms into the future
    Timestamp future = 100000000;  // 100ms
    MotionState predicted = integrator.predictToTime(current, future);

    // Should have moved ~0.1m in X
    ASSERT_NEAR(toFloat(predicted.pose.position.x), 0.1f, 0.01f);

    // Should have rotated ~0.01 rad around Z
    Vec3 rotVec = predicted.pose.orientation.toRotationVector();
    ASSERT_NEAR(toFloat(rotVec.z), 0.01f, 0.001f);
}

int main() {
    std::cout << "Running IMU tests...\n\n";

    RUN_TEST(preintegration_stationary);
    RUN_TEST(preintegration_rotation);
    RUN_TEST(integrator_buffer);
    RUN_TEST(integrator_prediction);
    RUN_TEST(bias_handling);
    RUN_TEST(predict_to_time);

    std::cout << "\nAll tests passed!\n";
    return 0;
}
