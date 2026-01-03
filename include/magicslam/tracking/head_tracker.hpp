#pragma once

#include "../core/types.hpp"
#include "../imu/imu_integrator.hpp"
#include <deque>
#include <mutex>

namespace MagicSLAM {

// Head tracking prediction for AR latency compensation
// Combines IMU prediction with velocity filtering for smooth output
class HeadTracker {
public:
    struct Config {
        // Prediction settings
        float maxPredictionTime = 0.050f;     // Max 50ms prediction
        float velocityDecay = 0.95f;          // Per-frame velocity decay
        float angularVelocityDecay = 0.90f;   // Angular velocity decay

        // Filtering settings
        float positionFilterAlpha = 0.8f;     // Position EMA filter
        float orientationFilterAlpha = 0.9f;  // Orientation filter
        float velocityFilterAlpha = 0.7f;     // Velocity filter

        // Jitter reduction
        float positionJitterThreshold = 0.001f;   // 1mm
        float orientationJitterThreshold = 0.001f; // ~0.06 degrees

        // Stability thresholds
        float stationaryThreshold = 0.01f;    // Consider stationary below this velocity
        float stationaryAngularThreshold = 0.02f;
    };

    explicit HeadTracker(const Config& config = {}) : config_(config) {
        reset();
    }

    void reset() {
        currentState_ = MotionState{};
        filteredState_ = MotionState{};
        predictedState_ = MotionState{};
        velocityHistory_.clear();
        angularVelocityHistory_.clear();
        isStationary_ = true;
        lastUpdateTime_ = 0;
    }

    // Update with new tracked pose from SLAM
    void update(const Pose& pose, const Vec3& velocity, const Vec3& angularVelocity,
                Timestamp timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Compute delta time
        float dt = lastUpdateTime_ > 0 ?
            (timestamp - lastUpdateTime_) * 1e-9f : 0.0f;

        // Store raw state
        currentState_.pose = pose;
        currentState_.velocity = velocity;
        currentState_.angularVelocity = angularVelocity;
        currentState_.timestamp = timestamp;

        // Apply filtering
        if (dt > 0 && dt < 0.5f) {
            filterState(dt);
        } else {
            // First update or large gap - use raw values
            filteredState_ = currentState_;
        }

        // Update velocity history for prediction
        updateVelocityHistory(velocity, angularVelocity);

        // Check if stationary
        float speed = velocity.norm();
        float angSpeed = angularVelocity.norm();
        isStationary_ = (speed < config_.stationaryThreshold &&
                        angSpeed < config_.stationaryAngularThreshold);

        lastUpdateTime_ = timestamp;
    }

    // Update with IMU measurement for high-frequency prediction
    void updateIMU(const ImuMeasurement& imu, const ImuBias& bias) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (lastUpdateTime_ == 0) return;

        float dt = (imu.timestamp - lastImuTime_) * 1e-9f;
        if (dt <= 0 || dt > 0.1f) {
            lastImuTime_ = imu.timestamp;
            return;
        }

        // Integrate angular velocity for orientation prediction
        Vec3 gyro = imu.angularVelocity - bias.gyroscope;
        Vec3 angleIncrement = gyro * dt;

        // Update predicted angular velocity with low-pass filter
        float alpha = 0.3f;
        imuAngularVelocity_.x = alpha * gyro.x + (1 - alpha) * imuAngularVelocity_.x;
        imuAngularVelocity_.y = alpha * gyro.y + (1 - alpha) * imuAngularVelocity_.y;
        imuAngularVelocity_.z = alpha * gyro.z + (1 - alpha) * imuAngularVelocity_.z;

        lastImuTime_ = imu.timestamp;
    }

    // Get predicted pose at future timestamp (for render time)
    Pose getPredictedPose(Timestamp targetTime) const {
        std::lock_guard<std::mutex> lock(mutex_);

        if (filteredState_.timestamp == 0) {
            return Pose{};
        }

        float dt = (targetTime - filteredState_.timestamp) * 1e-9f;

        // Clamp prediction time
        dt = std::max(0.0f, std::min(dt, config_.maxPredictionTime));

        if (isStationary_ || dt <= 0) {
            return filteredState_.pose;
        }

        // Get smoothed velocity for prediction
        Vec3 predVelocity = getSmoothedVelocity();
        Vec3 predAngularVelocity = getSmoothedAngularVelocity();

        // Use IMU angular velocity if available (more responsive)
        if (lastImuTime_ > filteredState_.timestamp) {
            predAngularVelocity = imuAngularVelocity_;
        }

        // Predict position with velocity decay
        float decay = std::pow(config_.velocityDecay, dt * 60.0f);
        Vec3 avgVelocity = predVelocity * (1.0f - decay) / (1.0f - config_.velocityDecay + 0.001f);
        Vec3 predictedPosition = filteredState_.pose.position + avgVelocity * dt;

        // Predict orientation with angular velocity
        float angDecay = std::pow(config_.angularVelocityDecay, dt * 60.0f);
        Vec3 avgAngularVelocity = predAngularVelocity * angDecay;
        Vec3 angleIncrement = avgAngularVelocity * dt;

        Quaternion dq = Quaternion::fromRotationVector(angleIncrement);
        Quaternion predictedOrientation = filteredState_.pose.orientation * dq;
        predictedOrientation = predictedOrientation.normalized();

        return Pose{predictedPosition, predictedOrientation};
    }

    // Get current filtered state
    const MotionState& getFilteredState() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return filteredState_;
    }

    // Check if head is stationary
    bool isStationary() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return isStationary_;
    }

    // Get prediction confidence (0-1)
    float getPredictionConfidence(Timestamp targetTime) const {
        std::lock_guard<std::mutex> lock(mutex_);

        if (filteredState_.timestamp == 0) return 0.0f;

        float dt = (targetTime - filteredState_.timestamp) * 1e-9f;
        if (dt <= 0) return 1.0f;

        // Confidence decreases with prediction time
        float maxDt = config_.maxPredictionTime;
        float confidence = 1.0f - std::min(1.0f, dt / maxDt);

        // Lower confidence when moving fast
        float speed = filteredState_.velocity.norm();
        if (speed > 1.0f) {
            confidence *= 0.8f;
        }

        return confidence;
    }

private:
    Config config_;
    mutable std::mutex mutex_;

    MotionState currentState_;
    MotionState filteredState_;
    MotionState predictedState_;

    std::deque<Vec3> velocityHistory_;
    std::deque<Vec3> angularVelocityHistory_;
    static constexpr size_t HISTORY_SIZE = 5;

    Vec3 imuAngularVelocity_;
    Timestamp lastImuTime_ = 0;
    Timestamp lastUpdateTime_ = 0;
    bool isStationary_ = true;

    void filterState(float dt) {
        float posAlpha = config_.positionFilterAlpha;
        float oriAlpha = config_.orientationFilterAlpha;
        float velAlpha = config_.velocityFilterAlpha;

        // Filter position with jitter reduction
        Vec3 posDiff = currentState_.pose.position - filteredState_.pose.position;
        if (posDiff.norm() > config_.positionJitterThreshold) {
            filteredState_.pose.position.x = posAlpha * currentState_.pose.position.x +
                (1 - posAlpha) * filteredState_.pose.position.x;
            filteredState_.pose.position.y = posAlpha * currentState_.pose.position.y +
                (1 - posAlpha) * filteredState_.pose.position.y;
            filteredState_.pose.position.z = posAlpha * currentState_.pose.position.z +
                (1 - posAlpha) * filteredState_.pose.position.z;
        }

        // Filter orientation with slerp
        Quaternion oriDiff = currentState_.pose.orientation *
            filteredState_.pose.orientation.inverse();
        float angle = 2.0f * std::acos(std::min(1.0f, std::abs(oriDiff.w)));

        if (angle > config_.orientationJitterThreshold) {
            filteredState_.pose.orientation = Quaternion::slerp(
                filteredState_.pose.orientation,
                currentState_.pose.orientation,
                oriAlpha);
        }

        // Filter velocity
        filteredState_.velocity.x = velAlpha * currentState_.velocity.x +
            (1 - velAlpha) * filteredState_.velocity.x;
        filteredState_.velocity.y = velAlpha * currentState_.velocity.y +
            (1 - velAlpha) * filteredState_.velocity.y;
        filteredState_.velocity.z = velAlpha * currentState_.velocity.z +
            (1 - velAlpha) * filteredState_.velocity.z;

        // Filter angular velocity
        filteredState_.angularVelocity.x = velAlpha * currentState_.angularVelocity.x +
            (1 - velAlpha) * filteredState_.angularVelocity.x;
        filteredState_.angularVelocity.y = velAlpha * currentState_.angularVelocity.y +
            (1 - velAlpha) * filteredState_.angularVelocity.y;
        filteredState_.angularVelocity.z = velAlpha * currentState_.angularVelocity.z +
            (1 - velAlpha) * filteredState_.angularVelocity.z;

        filteredState_.timestamp = currentState_.timestamp;
    }

    void updateVelocityHistory(const Vec3& velocity, const Vec3& angularVelocity) {
        velocityHistory_.push_back(velocity);
        angularVelocityHistory_.push_back(angularVelocity);

        while (velocityHistory_.size() > HISTORY_SIZE) {
            velocityHistory_.pop_front();
        }
        while (angularVelocityHistory_.size() > HISTORY_SIZE) {
            angularVelocityHistory_.pop_front();
        }
    }

    Vec3 getSmoothedVelocity() const {
        if (velocityHistory_.empty()) return Vec3{};

        // Use weighted average with more recent samples weighted higher
        Vec3 sum{};
        float totalWeight = 0;

        for (size_t i = 0; i < velocityHistory_.size(); ++i) {
            float weight = static_cast<float>(i + 1);
            sum += velocityHistory_[i] * weight;
            totalWeight += weight;
        }

        return sum * (1.0f / totalWeight);
    }

    Vec3 getSmoothedAngularVelocity() const {
        if (angularVelocityHistory_.empty()) return Vec3{};

        Vec3 sum{};
        float totalWeight = 0;

        for (size_t i = 0; i < angularVelocityHistory_.size(); ++i) {
            float weight = static_cast<float>(i + 1);
            sum += angularVelocityHistory_[i] * weight;
            totalWeight += weight;
        }

        return sum * (1.0f / totalWeight);
    }
};

// Asynchronous Time Warp (ATW) helper for AR glasses
// Provides pose correction for display latency
class AsyncTimeWarp {
public:
    struct Config {
        float displayLatency = 0.015f;  // 15ms typical display latency
        int predictionQueueSize = 3;    // Number of predictions to average
    };

    explicit AsyncTimeWarp(const Config& config = {}) : config_(config) {}

    // Record a prediction for later validation
    void recordPrediction(Timestamp predictedTime, const Pose& predictedPose) {
        std::lock_guard<std::mutex> lock(mutex_);

        predictions_.push_back({predictedTime, predictedPose});

        while (predictions_.size() > 100) {
            predictions_.pop_front();
        }
    }

    // Get warp transform to correct for prediction error
    Pose getWarpTransform(Timestamp renderTime, const Pose& currentPose) const {
        std::lock_guard<std::mutex> lock(mutex_);

        // Find prediction closest to render time
        const PredictionRecord* best = nullptr;
        Timestamp bestDiff = std::numeric_limits<Timestamp>::max();

        for (const auto& pred : predictions_) {
            Timestamp diff = std::abs(static_cast<int64_t>(pred.time) -
                                     static_cast<int64_t>(renderTime));
            if (diff < bestDiff) {
                bestDiff = diff;
                best = &pred;
            }
        }

        if (!best || bestDiff > 100000000) {  // 100ms
            return Pose{};  // Identity transform
        }

        // Compute correction transform
        // warp = current * predicted^-1
        Pose warp;
        warp.position = currentPose.position - best->pose.position;
        warp.orientation = currentPose.orientation * best->pose.orientation.inverse();

        return warp;
    }

    // Apply warp to rendered content
    static Pose applyWarp(const Pose& objectPose, const Pose& warp) {
        // Transform object pose by warp
        return Pose{
            objectPose.position + warp.orientation.rotate(warp.position),
            warp.orientation * objectPose.orientation
        };
    }

private:
    struct PredictionRecord {
        Timestamp time;
        Pose pose;
    };

    Config config_;
    mutable std::mutex mutex_;
    std::deque<PredictionRecord> predictions_;
};

// Motion-to-photon latency estimator
class LatencyEstimator {
public:
    void recordSensorTime(Timestamp sensorTime) {
        sensorTime_ = sensorTime;
    }

    void recordTrackingTime(Timestamp trackingTime) {
        trackingLatency_ = (trackingTime - sensorTime_) * 1e-9f;
    }

    void recordRenderTime(Timestamp renderTime) {
        renderLatency_ = (renderTime - sensorTime_) * 1e-9f;
    }

    void recordDisplayTime(Timestamp displayTime) {
        displayLatency_ = (displayTime - sensorTime_) * 1e-9f;

        // Update running average
        totalLatencyAvg_ = 0.9f * totalLatencyAvg_ + 0.1f * displayLatency_;
    }

    float getTotalLatency() const { return totalLatencyAvg_; }
    float getTrackingLatency() const { return trackingLatency_; }
    float getRenderLatency() const { return renderLatency_; }
    float getDisplayLatency() const { return displayLatency_; }

    // Get recommended prediction time
    float getRecommendedPredictionTime() const {
        return totalLatencyAvg_ + 0.005f;  // Add 5ms margin
    }

private:
    Timestamp sensorTime_ = 0;
    float trackingLatency_ = 0;
    float renderLatency_ = 0;
    float displayLatency_ = 0;
    float totalLatencyAvg_ = 0.020f;  // Initial estimate: 20ms
};

}  // namespace MagicSLAM
