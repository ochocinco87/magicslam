#pragma once

#include "core/types.hpp"
#include "core/feature.hpp"
#include "imu/imu_integrator.hpp"
#include "vision/feature_extractor.hpp"
#include "fusion/vio_ekf.hpp"
#include "mapping/map_manager.hpp"
#include "tracking/head_tracker.hpp"
#include "anchoring/anchor_system.hpp"
#include "platform/ar1_optimizations.hpp"

#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>

namespace MagicSLAM {

// SLAM System Configuration
struct Config {
    // Camera settings
    CameraIntrinsics camera;

    // Feature extraction
    FeatureExtractorConfig features;

    // IMU settings
    struct {
        Vec3 gravity{0, 0, -9.81f};
        float accelNoise = 0.01f;
        float gyroNoise = 0.001f;
    } imu;

    // Tracking settings
    struct {
        int minTrackedPoints = 20;
        float lostThreshold = 0.3f;         // Confidence below this = lost
        float relocalizationTimeout = 5.0f;  // seconds
    } tracking;

    // Map settings
    MapManager::Config map;

    // Head tracker settings
    HeadTracker::Config headTracker;

    // Anchor settings
    AnchorSystem::Config anchors;

    // Performance settings
    Platform::PowerManager::PerformanceMode powerMode =
        Platform::PowerManager::PerformanceMode::BALANCED;
};

// Tracking state
enum class TrackingState {
    NOT_INITIALIZED,
    INITIALIZING,
    TRACKING,
    LOST,
    RELOCALIZATION
};

// Callback types
using TrackingCallback = std::function<void(TrackingState, const Pose&, Timestamp)>;
using AnchorCallback = std::function<void(int64_t, const Anchor&)>;
using PlaneCallback = std::function<void(const std::vector<Plane>&)>;

// Main SLAM System class
class SlamSystem {
public:
    explicit SlamSystem(const Config& config = {})
        : config_(config),
          state_(TrackingState::NOT_INITIALIZED),
          currentKeyframeId_(-1),
          lastFrameTimestamp_(0) {

        // Initialize components
        featureExtractor_ = std::make_unique<FeatureExtractor>(config.features);
        imuIntegrator_ = std::make_unique<ImuIntegrator>();
        ekf_ = std::make_unique<VioEkf>();
        mapManager_ = std::make_unique<MapManager>(config.map);
        headTracker_ = std::make_unique<HeadTracker>(config.headTracker);
        anchorSystem_ = std::make_unique<AnchorSystem>(config.anchors);
        planeDetector_ = std::make_unique<PlaneDetector>();
        matcher_ = std::make_unique<FeatureMatcher>(50, 0.7f);

        // Set gravity
        imuIntegrator_->setGravity(config.imu.gravity);

        // Set power mode
        Platform::PowerManager::setMode(config.powerMode);
    }

    ~SlamSystem() {
        shutdown();
    }

    // Process IMU measurement (call at IMU rate, ~200Hz)
    void processIMU(const ImuMeasurement& imu) {
        std::lock_guard<std::mutex> lock(imuMutex_);

        imuIntegrator_->addMeasurement(imu);
        ekf_->predictIMU(imu);
        headTracker_->updateIMU(imu, ekf_->getBias());
    }

    // Process camera frame (call at camera rate, ~30Hz)
    void processFrame(const uint8_t* imageData, int width, int height,
                     Timestamp timestamp) {
        std::lock_guard<std::mutex> lock(frameMutex_);

        // Extract features
        auto features = featureExtractor_->extract(imageData, width, height);

        // Track based on current state
        switch (state_) {
            case TrackingState::NOT_INITIALIZED:
                initialize(features, timestamp);
                break;

            case TrackingState::INITIALIZING:
                tryInitialize(features, timestamp);
                break;

            case TrackingState::TRACKING:
                track(features, timestamp);
                break;

            case TrackingState::LOST:
            case TrackingState::RELOCALIZATION:
                relocalize(features, timestamp);
                break;
        }

        lastFrameTimestamp_ = timestamp;
        lastFeatures_ = std::move(features);
    }

    // Get current pose (thread-safe)
    Pose getCurrentPose() const {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return ekf_->getState().pose;
    }

    // Get predicted pose for rendering at future timestamp
    Pose getPredictedPose(Timestamp renderTime) const {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return headTracker_->getPredictedPose(renderTime);
    }

    // Get current tracking state
    TrackingState getTrackingState() const {
        return state_.load();
    }

    // Get tracking confidence (0-1)
    float getTrackingConfidence() const {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (state_ != TrackingState::TRACKING) return 0.0f;
        return trackingConfidence_;
    }

    // Anchor management
    int64_t createAnchor(const Vec3& position) {
        Timestamp now = lastFrameTimestamp_;
        return anchorSystem_->createPointAnchor(position, now);
    }

    int64_t createPlaneAnchor(const Plane& plane) {
        Timestamp now = lastFrameTimestamp_;
        return anchorSystem_->createPlaneAnchor(plane, now);
    }

    const Anchor* getAnchor(int64_t id) const {
        return anchorSystem_->getAnchor(id);
    }

    std::vector<Anchor> getAnchors() const {
        return anchorSystem_->getActiveAnchors();
    }

    void removeAnchor(int64_t id) {
        anchorSystem_->removeAnchor(id);
    }

    // Get detected planes
    std::vector<Plane> getPlanes() const {
        return anchorSystem_->getPlanes();
    }

    // Get map statistics
    size_t getMapPointCount() const {
        return mapManager_->getMapPointCount();
    }

    size_t getKeyframeCount() const {
        return mapManager_->getKeyframeCount();
    }

    // Set callbacks
    void setTrackingCallback(TrackingCallback callback) {
        trackingCallback_ = std::move(callback);
    }

    void setAnchorCallback(AnchorCallback callback) {
        anchorCallback_ = std::move(callback);
    }

    void setPlaneCallback(PlaneCallback callback) {
        planeCallback_ = std::move(callback);
    }

    // Reset system
    void reset() {
        std::lock_guard<std::mutex> lock1(frameMutex_);
        std::lock_guard<std::mutex> lock2(imuMutex_);
        std::lock_guard<std::mutex> lock3(stateMutex_);

        state_ = TrackingState::NOT_INITIALIZED;
        ekf_->reset();
        headTracker_->reset();
        imuIntegrator_->resetPreintegration();
        mapManager_ = std::make_unique<MapManager>(config_.map);
        anchorSystem_ = std::make_unique<AnchorSystem>(config_.anchors);

        currentKeyframeId_ = -1;
        lastFrameTimestamp_ = 0;
        lastFeatures_.clear();
        trackingConfidence_ = 0;
    }

    // Shutdown
    void shutdown() {
        running_ = false;
    }

private:
    Config config_;
    std::atomic<TrackingState> state_;

    // Components
    std::unique_ptr<FeatureExtractor> featureExtractor_;
    std::unique_ptr<ImuIntegrator> imuIntegrator_;
    std::unique_ptr<VioEkf> ekf_;
    std::unique_ptr<MapManager> mapManager_;
    std::unique_ptr<HeadTracker> headTracker_;
    std::unique_ptr<AnchorSystem> anchorSystem_;
    std::unique_ptr<PlaneDetector> planeDetector_;
    std::unique_ptr<FeatureMatcher> matcher_;

    // State
    int64_t currentKeyframeId_;
    Timestamp lastFrameTimestamp_;
    std::vector<Feature> lastFeatures_;
    float trackingConfidence_ = 0;

    // Thread safety
    mutable std::mutex frameMutex_;
    mutable std::mutex imuMutex_;
    mutable std::mutex stateMutex_;
    std::atomic<bool> running_{true};

    // Callbacks
    TrackingCallback trackingCallback_;
    AnchorCallback anchorCallback_;
    PlaneCallback planeCallback_;

    // Initialization data
    std::vector<Feature> initFeatures_;
    Pose initPose_;
    Timestamp initTimestamp_ = 0;

    void initialize(const std::vector<Feature>& features, Timestamp timestamp) {
        if (features.size() < static_cast<size_t>(config_.tracking.minTrackedPoints)) {
            return;
        }

        // Store first frame features
        initFeatures_ = features;
        initTimestamp_ = timestamp;

        // Initialize EKF at origin
        ekf_->initialize(Pose{}, Vec3{}, timestamp);

        state_ = TrackingState::INITIALIZING;
        notifyTrackingState(TrackingState::INITIALIZING, Pose{}, timestamp);
    }

    void tryInitialize(const std::vector<Feature>& features, Timestamp timestamp) {
        if (features.size() < static_cast<size_t>(config_.tracking.minTrackedPoints)) {
            return;
        }

        // Match features with initial frame
        auto matches = matcher_->match(features, initFeatures_);

        if (matches.size() < 30) {
            return;  // Not enough matches
        }

        // Check sufficient motion (for triangulation)
        float avgDisplacement = 0;
        for (const auto& m : matches) {
            float dx = features[m.queryIdx].position.x - initFeatures_[m.trainIdx].position.x;
            float dy = features[m.queryIdx].position.y - initFeatures_[m.trainIdx].position.y;
            avgDisplacement += std::sqrt(dx * dx + dy * dy);
        }
        avgDisplacement /= matches.size();

        if (avgDisplacement < 20.0f) {
            return;  // Not enough motion
        }

        // Estimate pose using essential matrix (simplified)
        Pose currentPose = ekf_->getState().pose;

        // Create initial keyframe
        KeyFrame kf;
        kf.timestamp = initTimestamp_;
        kf.pose = Pose{};  // Origin
        kf.features = initFeatures_;
        kf.mapPointIds.resize(initFeatures_.size(), -1);
        currentKeyframeId_ = mapManager_->addKeyframe(kf);

        // Triangulate initial map points
        for (const auto& m : matches) {
            // Simple forward projection for initial points
            Vec3 ray = config_.camera.unproject(features[m.queryIdx].position);
            Vec3 point = currentPose.position + currentPose.orientation.rotate(ray) * 2.0f;

            int64_t mpId = mapManager_->createMapPoint(
                point, features[m.queryIdx].descriptor, currentKeyframeId_);

            // This is simplified - real implementation would do proper triangulation
        }

        // Create second keyframe
        kf.timestamp = timestamp;
        kf.pose = currentPose;
        kf.features = features;
        kf.mapPointIds.resize(features.size(), -1);
        currentKeyframeId_ = mapManager_->addKeyframe(kf);

        state_ = TrackingState::TRACKING;
        trackingConfidence_ = 0.5f;

        // Update head tracker
        updateHeadTracker(currentPose, timestamp);

        notifyTrackingState(TrackingState::TRACKING, currentPose, timestamp);
    }

    void track(const std::vector<Feature>& features, Timestamp timestamp) {
        // Get predicted pose from EKF
        Pose predictedPose = ekf_->getState().pose;

        // Get local map points
        auto localMapPoints = mapManager_->getLocalMapPoints(currentKeyframeId_);

        // Match features to map points
        auto matches = matcher_->matchToMap(
            features, localMapPoints, predictedPose, config_.camera,
            featureExtractor_->getScaleFactors());

        int trackedPoints = static_cast<int>(matches.size());

        // Check if lost
        if (trackedPoints < config_.tracking.minTrackedPoints) {
            trackingConfidence_ *= 0.8f;
            if (trackingConfidence_ < config_.tracking.lostThreshold) {
                state_ = TrackingState::LOST;
                notifyTrackingState(TrackingState::LOST, predictedPose, timestamp);
                return;
            }
        } else {
            trackingConfidence_ = std::min(1.0f, trackingConfidence_ + 0.1f);
        }

        // Update EKF with visual observations
        std::vector<std::pair<Point2D, Vec3>> observations;
        for (const auto& m : matches) {
            observations.emplace_back(
                features[m.queryIdx].position,
                localMapPoints[m.trainIdx].position);

            mapManager_->incrementMapPointMatch(localMapPoints[m.trainIdx].id);
        }
        ekf_->updateMapPoints(observations, config_.camera);

        // Get updated pose
        Pose currentPose = ekf_->getState().pose;

        // Update head tracker
        updateHeadTracker(currentPose, timestamp);

        // Check if new keyframe needed
        if (mapManager_->needNewKeyframe(currentPose, trackedPoints, currentKeyframeId_)) {
            createKeyframe(features, currentPose, timestamp);
        }

        // Update anchors
        anchorSystem_->processMapPoints(localMapPoints, timestamp);

        // Detect planes periodically
        static int planeDetectionCounter = 0;
        if (++planeDetectionCounter >= 30) {  // Every 30 frames
            planeDetectionCounter = 0;
            detectPlanes();
        }

        notifyTrackingState(TrackingState::TRACKING, currentPose, timestamp);
    }

    void relocalize(const std::vector<Feature>& features, Timestamp timestamp) {
        // Try to match features against all keyframes
        // This is a simplified relocalization

        float bestScore = 0;
        int64_t bestKeyframeId = -1;

        // Search recent keyframes
        for (size_t i = 0; i < mapManager_->getKeyframeCount(); ++i) {
            // In real implementation, would use bag-of-words or similar
            // for efficient keyframe search
        }

        if (bestKeyframeId >= 0 && bestScore > 0.5f) {
            // Relocalized!
            state_ = TrackingState::TRACKING;
            currentKeyframeId_ = bestKeyframeId;
            trackingConfidence_ = bestScore;
        }

        // After timeout, reset
        float elapsed = (timestamp - lastFrameTimestamp_) * 1e-9f;
        if (elapsed > config_.tracking.relocalizationTimeout) {
            reset();
        }
    }

    void createKeyframe(const std::vector<Feature>& features,
                       const Pose& pose, Timestamp timestamp) {
        KeyFrame kf;
        kf.timestamp = timestamp;
        kf.pose = pose;
        kf.features = features;
        kf.mapPointIds.resize(features.size(), -1);

        // Get IMU preintegration
        const auto& preint = imuIntegrator_->getPreintegration();
        kf.preintegratedPosition = preint.getDeltaPosition();
        kf.preintegratedVelocity = preint.getDeltaVelocity();
        kf.preintegratedRotation = preint.getDeltaRotation();

        currentKeyframeId_ = mapManager_->addKeyframe(kf);

        // Reset IMU preintegration
        imuIntegrator_->resetPreintegration();

        // Triangulate new map points with previous keyframe
        // (simplified - real implementation would be more sophisticated)

        // Run local bundle adjustment in background
        // mapManager_->localBundleAdjustment(currentKeyframeId_, config_.camera);

        // Cull bad map points
        mapManager_->cullMapPoints();
    }

    void detectPlanes() {
        auto mapPoints = mapManager_->getAllMapPointPositions();
        if (mapPoints.size() < 20) return;

        auto detectedPlanes = planeDetector_->detectPlanes(mapPoints);
        anchorSystem_->updatePlanes(detectedPlanes);

        if (planeCallback_) {
            planeCallback_(anchorSystem_->getPlanes());
        }
    }

    void updateHeadTracker(const Pose& pose, Timestamp timestamp) {
        const auto& state = ekf_->getState();
        headTracker_->update(
            pose,
            state.velocity,
            state.angularVelocity,
            timestamp);
    }

    void notifyTrackingState(TrackingState newState, const Pose& pose, Timestamp timestamp) {
        if (trackingCallback_) {
            trackingCallback_(newState, pose, timestamp);
        }
    }
};

}  // namespace MagicSLAM
