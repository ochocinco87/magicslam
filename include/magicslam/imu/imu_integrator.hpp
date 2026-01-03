#pragma once

#include "../core/types.hpp"
#include <deque>
#include <mutex>

namespace MagicSLAM {

// IMU preintegration between keyframes
// Uses first-order integration for speed on embedded platforms
class ImuPreintegration {
public:
    ImuPreintegration() { reset(); }

    void reset() {
        deltaPosition = Vec3{};
        deltaVelocity = Vec3{};
        deltaRotation = Quaternion{};
        deltaBiasAcc = Vec3{};
        deltaBiasGyro = Vec3{};
        dt = 0;
        measurements.clear();

        // Reset covariance (simplified 15x15 state)
        std::fill(covariance.begin(), covariance.end(), 0.0f);
    }

    // Integrate a single IMU measurement
    void integrate(const ImuMeasurement& imu, const ImuBias& bias, float deltaT) {
        // Remove bias
        Vec3 acc = imu.acceleration - bias.accelerometer;
        Vec3 gyro = imu.angularVelocity - bias.gyroscope;

        // Store for re-integration if bias changes
        measurements.push_back(imu);

        // Rotation update (first-order)
        Vec3 angleIncrement = gyro * deltaT;
        Quaternion dq = Quaternion::fromRotationVector(angleIncrement);

        // Rotate acceleration to world frame
        Vec3 accWorld = deltaRotation.rotate(acc);

        // Position and velocity update
        deltaPosition += deltaVelocity * deltaT + accWorld * (0.5f * deltaT * deltaT);
        deltaVelocity += accWorld * deltaT;
        deltaRotation = deltaRotation * dq;
        deltaRotation = deltaRotation.normalized();

        dt += deltaT;

        // Update covariance (simplified diagonal approximation for speed)
        updateCovariance(acc, gyro, deltaT);
    }

    // Get predicted state given initial conditions
    MotionState predict(const MotionState& initial, const Vec3& gravity) const {
        MotionState result;
        result.timestamp = initial.timestamp + static_cast<Timestamp>(dt * 1e9);

        // Apply gravity
        Vec3 gravityDelta = gravity * (0.5f * dt * dt);
        Vec3 gravityVelDelta = gravity * dt;

        // Position
        result.pose.position = initial.pose.position +
            initial.pose.orientation.rotate(deltaPosition) +
            initial.velocity * dt +
            gravityDelta;

        // Velocity
        result.velocity = initial.velocity +
            initial.pose.orientation.rotate(deltaVelocity) +
            gravityVelDelta;

        // Orientation
        result.pose.orientation = initial.pose.orientation * deltaRotation;
        result.pose.orientation = result.pose.orientation.normalized();

        // Angular velocity from last measurement
        if (!measurements.empty()) {
            result.angularVelocity = measurements.back().angularVelocity;
        }

        return result;
    }

    // Re-integrate with updated bias (for optimization)
    void reintegrate(const ImuBias& newBias) {
        auto savedMeasurements = measurements;
        reset();
        for (size_t i = 1; i < savedMeasurements.size(); ++i) {
            float deltaT = (savedMeasurements[i].timestamp -
                           savedMeasurements[i-1].timestamp) * 1e-9f;
            integrate(savedMeasurements[i], newBias, deltaT);
        }
    }

    // Accessors
    const Vec3& getDeltaPosition() const { return deltaPosition; }
    const Vec3& getDeltaVelocity() const { return deltaVelocity; }
    const Quaternion& getDeltaRotation() const { return deltaRotation; }
    float getDeltaTime() const { return dt; }
    const std::array<float, 225>& getCovariance() const { return covariance; }

private:
    Vec3 deltaPosition;
    Vec3 deltaVelocity;
    Quaternion deltaRotation;
    Vec3 deltaBiasAcc;
    Vec3 deltaBiasGyro;
    float dt;

    std::vector<ImuMeasurement> measurements;
    std::array<float, 225> covariance;  // 15x15 covariance matrix

    // Noise parameters
    static constexpr float ACC_NOISE = 0.01f;      // m/s^2/sqrt(Hz)
    static constexpr float GYRO_NOISE = 0.001f;    // rad/s/sqrt(Hz)
    static constexpr float ACC_BIAS_NOISE = 0.0001f;
    static constexpr float GYRO_BIAS_NOISE = 0.00001f;

    void updateCovariance(const Vec3& acc, const Vec3& gyro, float dt) {
        // Simplified diagonal covariance update for embedded performance
        // Full covariance propagation would use Jacobians
        float dt2 = dt * dt;

        // Position uncertainty grows with velocity uncertainty
        covariance[0] += ACC_NOISE * ACC_NOISE * dt2 * dt2 * 0.25f;
        covariance[16] += ACC_NOISE * ACC_NOISE * dt2 * dt2 * 0.25f;
        covariance[32] += ACC_NOISE * ACC_NOISE * dt2 * dt2 * 0.25f;

        // Velocity uncertainty
        covariance[48] += ACC_NOISE * ACC_NOISE * dt2;
        covariance[64] += ACC_NOISE * ACC_NOISE * dt2;
        covariance[80] += ACC_NOISE * ACC_NOISE * dt2;

        // Rotation uncertainty
        covariance[96] += GYRO_NOISE * GYRO_NOISE * dt2;
        covariance[112] += GYRO_NOISE * GYRO_NOISE * dt2;
        covariance[128] += GYRO_NOISE * GYRO_NOISE * dt2;

        // Bias random walk
        covariance[144] += ACC_BIAS_NOISE * ACC_BIAS_NOISE * dt;
        covariance[160] += ACC_BIAS_NOISE * ACC_BIAS_NOISE * dt;
        covariance[176] += ACC_BIAS_NOISE * ACC_BIAS_NOISE * dt;
        covariance[192] += GYRO_BIAS_NOISE * GYRO_BIAS_NOISE * dt;
        covariance[208] += GYRO_BIAS_NOISE * GYRO_BIAS_NOISE * dt;
        covariance[224] += GYRO_BIAS_NOISE * GYRO_BIAS_NOISE * dt;
    }
};

// IMU buffer and high-frequency integrator
class ImuIntegrator {
public:
    ImuIntegrator() : gravity_{0, 0, -9.81f} {}

    void setGravity(const Vec3& g) { gravity_ = g; }

    void addMeasurement(const ImuMeasurement& imu) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!measurements_.empty()) {
            float dt = (imu.timestamp - measurements_.back().timestamp) * 1e-9f;
            if (dt > 0 && dt < 0.5f) {  // Sanity check
                preintegration_.integrate(imu, currentBias_, dt);
            }
        }

        measurements_.push_back(imu);

        // Limit buffer size
        while (measurements_.size() > MAX_BUFFER_SIZE) {
            measurements_.pop_front();
        }
    }

    void setBias(const ImuBias& bias) {
        std::lock_guard<std::mutex> lock(mutex_);
        currentBias_ = bias;
    }

    // Get current motion state prediction
    MotionState predict(const MotionState& reference) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return preintegration_.predict(reference, gravity_);
    }

    // Predict to a future timestamp using velocity model
    MotionState predictToTime(const MotionState& current, Timestamp targetTime) const {
        std::lock_guard<std::mutex> lock(mutex_);

        float dt = (targetTime - current.timestamp) * 1e-9f;
        if (dt <= 0) return current;

        MotionState predicted;
        predicted.timestamp = targetTime;

        // Use current angular velocity for rotation prediction
        Vec3 angleIncrement = current.angularVelocity * dt;
        Quaternion dq = Quaternion::fromRotationVector(angleIncrement);
        predicted.pose.orientation = current.pose.orientation * dq;
        predicted.pose.orientation = predicted.pose.orientation.normalized();

        // Linear velocity prediction with gravity
        predicted.velocity = current.velocity + gravity_ * dt;
        predicted.pose.position = current.pose.position +
            current.velocity * dt + gravity_ * (0.5f * dt * dt);

        predicted.angularVelocity = current.angularVelocity;

        return predicted;
    }

    // Get preintegration for optimization
    const ImuPreintegration& getPreintegration() const { return preintegration_; }

    // Reset preintegration (after keyframe)
    void resetPreintegration() {
        std::lock_guard<std::mutex> lock(mutex_);
        preintegration_.reset();
    }

    // Get IMU measurements in time range
    std::vector<ImuMeasurement> getMeasurements(Timestamp start, Timestamp end) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<ImuMeasurement> result;
        for (const auto& m : measurements_) {
            if (m.timestamp >= start && m.timestamp <= end) {
                result.push_back(m);
            }
        }
        return result;
    }

    // Get latest measurement
    bool getLatest(ImuMeasurement& imu) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (measurements_.empty()) return false;
        imu = measurements_.back();
        return true;
    }

private:
    static constexpr size_t MAX_BUFFER_SIZE = 1000;  // ~5 seconds at 200Hz

    mutable std::mutex mutex_;
    std::deque<ImuMeasurement> measurements_;
    ImuPreintegration preintegration_;
    ImuBias currentBias_;
    Vec3 gravity_;
};

}  // namespace MagicSLAM
