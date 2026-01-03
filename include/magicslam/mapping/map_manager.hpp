#pragma once

#include "../core/types.hpp"
#include "../core/feature.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <algorithm>

namespace MagicSLAM {

// Covisibility graph for efficient keyframe lookup
class CovisibilityGraph {
public:
    void addKeyframe(int64_t kfId) {
        if (connections_.find(kfId) == connections_.end()) {
            connections_[kfId] = {};
        }
    }

    void addConnection(int64_t kf1, int64_t kf2, int weight) {
        connections_[kf1][kf2] = weight;
        connections_[kf2][kf1] = weight;
    }

    void removeKeyframe(int64_t kfId) {
        for (auto& [id, conn] : connections_) {
            conn.erase(kfId);
        }
        connections_.erase(kfId);
    }

    std::vector<int64_t> getConnectedKeyframes(int64_t kfId, int minWeight = 0) const {
        std::vector<int64_t> result;
        auto it = connections_.find(kfId);
        if (it != connections_.end()) {
            for (const auto& [id, weight] : it->second) {
                if (weight >= minWeight) {
                    result.push_back(id);
                }
            }
        }
        // Sort by weight (descending)
        std::sort(result.begin(), result.end(), [&](int64_t a, int64_t b) {
            return connections_.at(kfId).at(a) > connections_.at(kfId).at(b);
        });
        return result;
    }

    int getWeight(int64_t kf1, int64_t kf2) const {
        auto it1 = connections_.find(kf1);
        if (it1 != connections_.end()) {
            auto it2 = it1->second.find(kf2);
            if (it2 != it1->second.end()) {
                return it2->second;
            }
        }
        return 0;
    }

private:
    std::unordered_map<int64_t, std::unordered_map<int64_t, int>> connections_;
};

// Map management class
class MapManager {
public:
    struct Config {
        int maxKeyframes = 100;           // Maximum keyframes to keep
        int minMapPoints = 50;            // Minimum map points for tracking
        float keyframeTranslationThresh = 0.1f;  // meters
        float keyframeRotationThresh = 0.1f;     // radians
        int keyframeFeatureThresh = 20;          // Min features for new keyframe
        float cullMapPointObsRatio = 0.25f;      // Cull if obs/visible < ratio
        int localWindowSize = 10;                // Local BA window
    };

    explicit MapManager(const Config& config = {}) : config_(config), nextKeyframeId_(0), nextMapPointId_(0) {}

    // Add new keyframe
    int64_t addKeyframe(const KeyFrame& kf) {
        std::lock_guard<std::mutex> lock(mutex_);

        KeyFrame newKf = kf;
        newKf.id = nextKeyframeId_++;
        keyframes_[newKf.id] = newKf;
        covisibility_.addKeyframe(newKf.id);

        // Update covisibility graph
        updateCovisibility(newKf.id);

        // Cull old keyframes if needed
        if (keyframes_.size() > static_cast<size_t>(config_.maxKeyframes)) {
            cullKeyframes();
        }

        return newKf.id;
    }

    // Create new map point from triangulation
    int64_t createMapPoint(const Vec3& position, const OrbDescriptor& descriptor,
                          int64_t observingKeyframe) {
        std::lock_guard<std::mutex> lock(mutex_);

        MapPoint mp;
        mp.id = nextMapPointId_++;
        mp.position = position;
        mp.descriptor = descriptor;
        mp.observationCount = 1;
        mp.matchedCount = 1;
        mp.isBad = false;

        mapPoints_[mp.id] = mp;
        return mp.id;
    }

    // Triangulate new map point from two observations
    bool triangulateMapPoint(int64_t kf1Id, int featureIdx1,
                            int64_t kf2Id, int featureIdx2,
                            const CameraIntrinsics& camera) {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it1 = keyframes_.find(kf1Id);
        auto it2 = keyframes_.find(kf2Id);
        if (it1 == keyframes_.end() || it2 == keyframes_.end()) return false;

        const KeyFrame& kf1 = it1->second;
        const KeyFrame& kf2 = it2->second;

        if (featureIdx1 >= static_cast<int>(kf1.features.size()) ||
            featureIdx2 >= static_cast<int>(kf2.features.size())) return false;

        // Get rays in world frame
        Vec3 ray1 = camera.unproject(kf1.features[featureIdx1].position);
        Vec3 ray2 = camera.unproject(kf2.features[featureIdx2].position);
        ray1 = kf1.pose.orientation.rotate(ray1);
        ray2 = kf2.pose.orientation.rotate(ray2);

        // Check parallax
        float cosParallax = ray1.dot(ray2) / (ray1.norm() * ray2.norm());
        if (cosParallax > 0.9998f) return false;  // Too little parallax

        // Triangulate using midpoint method (fast for embedded)
        Vec3 t = kf2.pose.position - kf1.pose.position;
        float a = ray1.dot(ray1);
        float b = ray1.dot(ray2);
        float c = ray2.dot(ray2);
        float d = ray1.dot(t);
        float e = ray2.dot(t);

        float denom = a * c - b * b;
        if (std::abs(denom) < 1e-6f) return false;

        float s = (b * e - c * d) / denom;
        float tt = (a * e - b * d) / denom;

        if (s < 0 || tt < 0) return false;  // Behind cameras

        Vec3 p1 = kf1.pose.position + ray1 * s;
        Vec3 p2 = kf2.pose.position + ray2 * tt;
        Vec3 point = (p1 + p2) * 0.5f;

        // Check reprojection error
        Vec3 pCam1 = kf1.pose.inverseTransformPoint(point);
        Vec3 pCam2 = kf2.pose.inverseTransformPoint(point);
        if (pCam1.z <= 0 || pCam2.z <= 0) return false;

        Point2D proj1 = camera.project(pCam1);
        Point2D proj2 = camera.project(pCam2);

        float err1 = std::sqrt(
            std::pow(proj1.x - kf1.features[featureIdx1].position.x, 2) +
            std::pow(proj1.y - kf1.features[featureIdx1].position.y, 2));
        float err2 = std::sqrt(
            std::pow(proj2.x - kf2.features[featureIdx2].position.x, 2) +
            std::pow(proj2.y - kf2.features[featureIdx2].position.y, 2));

        if (err1 > 5.0f || err2 > 5.0f) return false;

        // Create map point
        int64_t mpId = createMapPointInternal(point, kf1.features[featureIdx1].descriptor);

        // Update keyframe observations
        keyframes_[kf1Id].mapPointIds[featureIdx1] = mpId;
        keyframes_[kf2Id].mapPointIds[featureIdx2] = mpId;

        return true;
    }

    // Check if new keyframe is needed
    bool needNewKeyframe(const Pose& currentPose, int trackedFeatures,
                        int64_t referenceKfId) const {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = keyframes_.find(referenceKfId);
        if (it == keyframes_.end()) return true;

        const KeyFrame& refKf = it->second;

        // Check translation
        Vec3 translation = currentPose.position - refKf.pose.position;
        float dist = translation.norm();
        if (dist > config_.keyframeTranslationThresh) return true;

        // Check rotation
        Quaternion dq = currentPose.orientation * refKf.pose.orientation.inverse();
        float angle = 2.0f * std::acos(std::min(1.0f, std::abs(dq.w)));
        if (angle > config_.keyframeRotationThresh) return true;

        // Check tracked features
        if (trackedFeatures < config_.keyframeFeatureThresh) return true;

        return false;
    }

    // Get local map points for tracking
    std::vector<MapPoint> getLocalMapPoints(int64_t keyframeId,
                                            int maxPoints = 500) const {
        std::lock_guard<std::mutex> lock(mutex_);

        std::unordered_set<int64_t> localMpIds;
        std::vector<int64_t> localKfIds;

        // Get covisible keyframes
        auto connected = covisibility_.getConnectedKeyframes(keyframeId, 5);
        localKfIds.push_back(keyframeId);
        for (auto id : connected) {
            if (localKfIds.size() >= static_cast<size_t>(config_.localWindowSize)) break;
            localKfIds.push_back(id);
        }

        // Collect map points from local keyframes
        for (auto kfId : localKfIds) {
            auto it = keyframes_.find(kfId);
            if (it != keyframes_.end()) {
                for (auto mpId : it->second.mapPointIds) {
                    if (mpId >= 0) {
                        localMpIds.insert(mpId);
                    }
                }
            }
        }

        // Return map points
        std::vector<MapPoint> result;
        result.reserve(std::min(localMpIds.size(), static_cast<size_t>(maxPoints)));

        for (auto mpId : localMpIds) {
            if (result.size() >= static_cast<size_t>(maxPoints)) break;
            auto it = mapPoints_.find(mpId);
            if (it != mapPoints_.end() && !it->second.isBad) {
                result.push_back(it->second);
            }
        }

        return result;
    }

    // Update map point after observation
    void updateMapPoint(int64_t mpId, const Vec3& newPosition,
                       const OrbDescriptor& descriptor) {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = mapPoints_.find(mpId);
        if (it != mapPoints_.end()) {
            it->second.position = newPosition;
            it->second.descriptor = descriptor;
            it->second.observationCount++;
        }
    }

    // Increment match count for map point
    void incrementMapPointMatch(int64_t mpId) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mapPoints_.find(mpId);
        if (it != mapPoints_.end()) {
            it->second.matchedCount++;
        }
    }

    // Cull bad map points
    void cullMapPoints() {
        std::lock_guard<std::mutex> lock(mutex_);

        std::vector<int64_t> toRemove;
        for (auto& [id, mp] : mapPoints_) {
            if (mp.isBad) {
                toRemove.push_back(id);
                continue;
            }

            // Check observation ratio
            if (mp.observationCount > 2) {
                float ratio = static_cast<float>(mp.matchedCount) / mp.observationCount;
                if (ratio < config_.cullMapPointObsRatio) {
                    mp.isBad = true;
                    toRemove.push_back(id);
                }
            }
        }

        for (auto id : toRemove) {
            mapPoints_.erase(id);
        }
    }

    // Local bundle adjustment (simplified for embedded)
    void localBundleAdjustment(int64_t keyframeId, const CameraIntrinsics& camera,
                               int iterations = 3) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Get local keyframes
        auto connected = covisibility_.getConnectedKeyframes(keyframeId, 5);
        std::vector<int64_t> localKfIds = {keyframeId};
        for (size_t i = 0; i < std::min(connected.size(),
             static_cast<size_t>(config_.localWindowSize - 1)); ++i) {
            localKfIds.push_back(connected[i]);
        }

        // Collect local map points
        std::unordered_set<int64_t> localMpIds;
        for (auto kfId : localKfIds) {
            for (auto mpId : keyframes_[kfId].mapPointIds) {
                if (mpId >= 0) localMpIds.insert(mpId);
            }
        }

        // Simple Gauss-Newton optimization
        for (int iter = 0; iter < iterations; ++iter) {
            // Optimize map point positions
            for (auto mpId : localMpIds) {
                auto& mp = mapPoints_[mpId];
                if (mp.isBad) continue;

                Vec3 sumCorrection{};
                int count = 0;

                for (auto kfId : localKfIds) {
                    const auto& kf = keyframes_[kfId];
                    for (size_t i = 0; i < kf.mapPointIds.size(); ++i) {
                        if (kf.mapPointIds[i] == mpId) {
                            // Compute reprojection error
                            Vec3 pCam = kf.pose.inverseTransformPoint(mp.position);
                            if (pCam.z <= 0) continue;

                            Point2D proj = camera.project(pCam);
                            float errX = kf.features[i].position.x - proj.x;
                            float errY = kf.features[i].position.y - proj.y;

                            // Approximate correction (gradient descent step)
                            float depth = toFloat(pCam.z);
                            Vec3 correction = kf.pose.orientation.rotate(Vec3{
                                errX * depth / camera.fx * 0.1f,
                                errY * depth / camera.fy * 0.1f,
                                0
                            });
                            sumCorrection += correction;
                            count++;
                        }
                    }
                }

                if (count > 0) {
                    mp.position += sumCorrection * (1.0f / count);
                }
            }
        }
    }

    // Getters
    const KeyFrame* getKeyframe(int64_t id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = keyframes_.find(id);
        return it != keyframes_.end() ? &it->second : nullptr;
    }

    const MapPoint* getMapPoint(int64_t id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mapPoints_.find(id);
        return it != mapPoints_.end() ? &it->second : nullptr;
    }

    size_t getKeyframeCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return keyframes_.size();
    }

    size_t getMapPointCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mapPoints_.size();
    }

    std::vector<Vec3> getAllMapPointPositions() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Vec3> positions;
        positions.reserve(mapPoints_.size());
        for (const auto& [id, mp] : mapPoints_) {
            if (!mp.isBad) {
                positions.push_back(mp.position);
            }
        }
        return positions;
    }

private:
    Config config_;
    mutable std::mutex mutex_;

    std::unordered_map<int64_t, KeyFrame> keyframes_;
    std::unordered_map<int64_t, MapPoint> mapPoints_;
    CovisibilityGraph covisibility_;

    int64_t nextKeyframeId_;
    int64_t nextMapPointId_;

    int64_t createMapPointInternal(const Vec3& position, const OrbDescriptor& descriptor) {
        MapPoint mp;
        mp.id = nextMapPointId_++;
        mp.position = position;
        mp.descriptor = descriptor;
        mp.observationCount = 2;
        mp.matchedCount = 2;
        mp.isBad = false;
        mapPoints_[mp.id] = mp;
        return mp.id;
    }

    void updateCovisibility(int64_t kfId) {
        const auto& kf = keyframes_[kfId];
        std::unordered_map<int64_t, int> sharedPoints;

        for (auto mpId : kf.mapPointIds) {
            if (mpId < 0) continue;

            // Find other keyframes observing this point
            for (const auto& [otherId, otherKf] : keyframes_) {
                if (otherId == kfId) continue;
                for (auto otherMpId : otherKf.mapPointIds) {
                    if (otherMpId == mpId) {
                        sharedPoints[otherId]++;
                        break;
                    }
                }
            }
        }

        // Add connections
        for (const auto& [otherId, count] : sharedPoints) {
            if (count >= 5) {
                covisibility_.addConnection(kfId, otherId, count);
            }
        }
    }

    void cullKeyframes() {
        // Find keyframes with most redundancy
        std::vector<std::pair<int64_t, float>> redundancy;

        for (const auto& [id, kf] : keyframes_) {
            if (id == 0) continue;  // Keep first keyframe

            // Count observations covered by other keyframes
            int coveredObs = 0;
            int totalObs = 0;

            for (auto mpId : kf.mapPointIds) {
                if (mpId < 0) continue;
                totalObs++;

                int otherKfCount = 0;
                for (const auto& [otherId, otherKf] : keyframes_) {
                    if (otherId == id) continue;
                    for (auto otherMpId : otherKf.mapPointIds) {
                        if (otherMpId == mpId) {
                            otherKfCount++;
                            break;
                        }
                    }
                }

                if (otherKfCount >= 3) {
                    coveredObs++;
                }
            }

            if (totalObs > 0) {
                float ratio = static_cast<float>(coveredObs) / totalObs;
                redundancy.emplace_back(id, ratio);
            }
        }

        // Remove most redundant keyframes
        std::sort(redundancy.begin(), redundancy.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        int toRemove = static_cast<int>(keyframes_.size()) - config_.maxKeyframes;
        for (int i = 0; i < toRemove && i < static_cast<int>(redundancy.size()); ++i) {
            if (redundancy[i].second > 0.9f) {
                int64_t kfId = redundancy[i].first;
                keyframes_.erase(kfId);
                covisibility_.removeKeyframe(kfId);
            }
        }
    }
};

}  // namespace MagicSLAM
