#pragma once

#include "../core/types.hpp"
#include "../core/feature.hpp"
#include <unordered_map>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cmath>

namespace MagicSLAM {

// Anchor types
enum class AnchorType {
    POINT,      // Single 3D point anchor
    PLANE,      // Planar surface anchor
    FEATURE,    // Feature-based anchor (tracks with features)
    SEMANTIC    // Semantic anchor (e.g., table, wall)
};

// Anchor state
enum class AnchorState {
    INITIALIZING,  // Gathering observations
    TRACKING,      // Actively tracked
    LIMITED,       // Limited tracking (few observations)
    LOST,          // Cannot track
    PAUSED         // Temporarily paused
};

// Detected plane
struct Plane {
    int64_t id;
    Vec3 center;          // Center point
    Vec3 normal;          // Normal vector (pointing towards camera)
    Vec3 extentX;         // X axis on plane
    Vec3 extentY;         // Y axis on plane
    float width;          // Extent in X
    float height;         // Extent in Y
    std::vector<Vec3> boundary;  // Convex hull boundary
    int observationCount;
    bool isVertical;      // Wall vs floor/ceiling

    Plane() : id(-1), width(0), height(0), observationCount(0), isVertical(false) {}

    // Check if point is on plane
    bool containsPoint(const Vec3& p, float tolerance = 0.05f) const {
        Vec3 toPoint = p - center;
        float dist = std::abs(toPoint.dot(normal));
        if (dist > tolerance) return false;

        // Check within bounds
        float projX = toPoint.dot(extentX);
        float projY = toPoint.dot(extentY);
        return std::abs(projX) <= width * 0.5f + tolerance &&
               std::abs(projY) <= height * 0.5f + tolerance;
    }

    // Project point onto plane
    Vec3 projectPoint(const Vec3& p) const {
        Vec3 toPoint = p - center;
        float dist = toPoint.dot(normal);
        return p - normal * dist;
    }
};

// Spatial anchor for virtual objects
struct Anchor {
    int64_t id;
    AnchorType type;
    AnchorState state;
    Pose pose;              // World pose of anchor
    float confidence;       // 0-1 confidence score
    Timestamp lastSeen;     // Last observation time
    int observationCount;

    // Associated data
    int64_t planeId;        // If plane anchor
    std::vector<int64_t> mapPointIds;  // Associated map points

    // Stability tracking
    Vec3 positionVariance;
    Quaternion orientationVariance;
    float stabilityScore;   // Higher = more stable

    Anchor() : id(-1), type(AnchorType::POINT), state(AnchorState::INITIALIZING),
               confidence(0), lastSeen(0), observationCount(0), planeId(-1),
               stabilityScore(0) {}

    // Get transform relative to anchor
    Pose getRelativeTransform(const Pose& worldPose) const {
        return pose.inverse() * worldPose;
    }

    // Apply anchor transform to get world pose
    Pose getWorldPose(const Pose& relativePose) const {
        return pose * relativePose;
    }
};

// Plane detector using RANSAC
class PlaneDetector {
public:
    struct Config {
        float distanceThreshold = 0.02f;    // 2cm inlier threshold
        int minInliers = 20;                 // Minimum points for plane
        int maxIterations = 100;             // RANSAC iterations
        float mergeAngleThreshold = 0.1f;   // Radians for plane merging
        float mergeDistThreshold = 0.1f;    // Meters for plane merging
        float minPlaneArea = 0.04f;         // 20cm x 20cm minimum
    };

    explicit PlaneDetector(const Config& config = {}) : config_(config), nextPlaneId_(0) {}

    // Detect planes from point cloud
    std::vector<Plane> detectPlanes(const std::vector<Vec3>& points) {
        std::vector<Plane> planes;
        std::vector<bool> used(points.size(), false);

        for (int attempt = 0; attempt < 5 && planes.size() < 10; ++attempt) {
            Plane plane;
            std::vector<int> inliers;

            if (ransacPlane(points, used, plane, inliers)) {
                // Mark points as used
                for (int idx : inliers) {
                    used[idx] = true;
                }

                // Refine plane and compute extent
                refinePlane(points, inliers, plane);

                plane.id = nextPlaneId_++;
                plane.observationCount = 1;
                planes.push_back(plane);
            }
        }

        // Merge similar planes
        mergePlanes(planes);

        return planes;
    }

    // Update existing plane with new observations
    void updatePlane(Plane& plane, const std::vector<Vec3>& newPoints) {
        // Find inliers
        std::vector<Vec3> inliers;
        for (const auto& p : newPoints) {
            Vec3 toPoint = p - plane.center;
            float dist = std::abs(toPoint.dot(plane.normal));
            if (dist < config_.distanceThreshold) {
                inliers.push_back(p);
            }
        }

        if (inliers.size() < 5) return;

        // Update center (weighted average)
        Vec3 newCenter{};
        for (const auto& p : inliers) {
            newCenter += p;
        }
        newCenter = newCenter * (1.0f / inliers.size());

        float alpha = 0.3f;
        plane.center = plane.center * (1 - alpha) + newCenter * alpha;

        // Update extent
        updatePlaneExtent(plane, inliers);

        plane.observationCount++;
    }

private:
    Config config_;
    int64_t nextPlaneId_;

    bool ransacPlane(const std::vector<Vec3>& points,
                    const std::vector<bool>& used,
                    Plane& outPlane,
                    std::vector<int>& outInliers) {
        if (points.size() < 3) return false;

        int bestInlierCount = 0;
        Vec3 bestNormal;
        Vec3 bestCenter;

        for (int iter = 0; iter < config_.maxIterations; ++iter) {
            // Sample 3 random unused points
            std::vector<int> sample;
            for (int i = 0; i < 100 && sample.size() < 3; ++i) {
                int idx = rand() % points.size();
                if (!used[idx]) {
                    sample.push_back(idx);
                }
            }
            if (sample.size() < 3) continue;

            // Compute plane
            Vec3 v1 = points[sample[1]] - points[sample[0]];
            Vec3 v2 = points[sample[2]] - points[sample[0]];
            Vec3 normal = v1.cross(v2);
            float len = normal.norm();
            if (len < 1e-6f) continue;
            normal = normal * (1.0f / len);

            Vec3 center = (points[sample[0]] + points[sample[1]] + points[sample[2]]) * (1.0f / 3.0f);

            // Count inliers
            int inlierCount = 0;
            for (size_t i = 0; i < points.size(); ++i) {
                if (used[i]) continue;
                Vec3 toPoint = points[i] - center;
                float dist = std::abs(toPoint.dot(normal));
                if (dist < config_.distanceThreshold) {
                    inlierCount++;
                }
            }

            if (inlierCount > bestInlierCount) {
                bestInlierCount = inlierCount;
                bestNormal = normal;
                bestCenter = center;
            }
        }

        if (bestInlierCount < config_.minInliers) return false;

        // Collect inliers
        outInliers.clear();
        for (size_t i = 0; i < points.size(); ++i) {
            if (used[i]) continue;
            Vec3 toPoint = points[i] - bestCenter;
            float dist = std::abs(toPoint.dot(bestNormal));
            if (dist < config_.distanceThreshold) {
                outInliers.push_back(static_cast<int>(i));
            }
        }

        outPlane.normal = bestNormal;
        outPlane.center = bestCenter;

        return true;
    }

    void refinePlane(const std::vector<Vec3>& points,
                    const std::vector<int>& inliers,
                    Plane& plane) {
        if (inliers.size() < 3) return;

        // Recompute center
        Vec3 center{};
        for (int idx : inliers) {
            center += points[idx];
        }
        center = center * (1.0f / inliers.size());
        plane.center = center;

        // PCA for better normal and extent axes
        // Compute covariance matrix
        float cov[3][3] = {{0}};
        for (int idx : inliers) {
            Vec3 d = points[idx] - center;
            cov[0][0] += toFloat(d.x * d.x);
            cov[0][1] += toFloat(d.x * d.y);
            cov[0][2] += toFloat(d.x * d.z);
            cov[1][1] += toFloat(d.y * d.y);
            cov[1][2] += toFloat(d.y * d.z);
            cov[2][2] += toFloat(d.z * d.z);
        }
        cov[1][0] = cov[0][1];
        cov[2][0] = cov[0][2];
        cov[2][1] = cov[1][2];

        // Simple power iteration for smallest eigenvector (normal)
        Vec3 v{1, 0, 0};
        for (int iter = 0; iter < 20; ++iter) {
            Vec3 newV{
                cov[0][0] * toFloat(v.x) + cov[0][1] * toFloat(v.y) + cov[0][2] * toFloat(v.z),
                cov[1][0] * toFloat(v.x) + cov[1][1] * toFloat(v.y) + cov[1][2] * toFloat(v.z),
                cov[2][0] * toFloat(v.x) + cov[2][1] * toFloat(v.y) + cov[2][2] * toFloat(v.z)
            };
            v = newV.normalized();
        }

        // Use original normal if similar (for consistency)
        if (v.dot(plane.normal) < 0) {
            v = v * -1.0f;
        }
        if (std::abs(v.dot(plane.normal)) > 0.9f) {
            // Keep refined normal
            plane.normal = v;
        }

        // Determine if vertical or horizontal
        Vec3 up{0, 0, 1};
        float upDot = std::abs(plane.normal.dot(up));
        plane.isVertical = upDot < 0.5f;

        // Compute extent axes
        if (plane.isVertical) {
            plane.extentY = up;
            plane.extentX = plane.normal.cross(up).normalized();
        } else {
            plane.extentX = Vec3{1, 0, 0};
            if (std::abs(plane.normal.dot(plane.extentX)) > 0.9f) {
                plane.extentX = Vec3{0, 1, 0};
            }
            plane.extentX = (plane.extentX - plane.normal * plane.extentX.dot(plane.normal)).normalized();
            plane.extentY = plane.normal.cross(plane.extentX);
        }

        // Compute extent
        updatePlaneExtent(plane, std::vector<Vec3>{});
        for (int idx : inliers) {
            Vec3 toPoint = points[idx] - plane.center;
            float projX = std::abs(toPoint.dot(plane.extentX));
            float projY = std::abs(toPoint.dot(plane.extentY));
            plane.width = std::max(plane.width, projX * 2);
            plane.height = std::max(plane.height, projY * 2);
        }
    }

    void updatePlaneExtent(Plane& plane, const std::vector<Vec3>& points) {
        for (const auto& p : points) {
            Vec3 toPoint = p - plane.center;
            float projX = std::abs(toPoint.dot(plane.extentX));
            float projY = std::abs(toPoint.dot(plane.extentY));
            plane.width = std::max(plane.width, projX * 2);
            plane.height = std::max(plane.height, projY * 2);
        }
    }

    void mergePlanes(std::vector<Plane>& planes) {
        std::vector<bool> merged(planes.size(), false);
        std::vector<Plane> result;

        for (size_t i = 0; i < planes.size(); ++i) {
            if (merged[i]) continue;

            Plane& p1 = planes[i];

            for (size_t j = i + 1; j < planes.size(); ++j) {
                if (merged[j]) continue;

                const Plane& p2 = planes[j];

                // Check normal similarity
                float normalDot = std::abs(p1.normal.dot(p2.normal));
                if (normalDot < std::cos(config_.mergeAngleThreshold)) continue;

                // Check distance
                Vec3 centerDiff = p2.center - p1.center;
                float dist = std::abs(centerDiff.dot(p1.normal));
                if (dist > config_.mergeDistThreshold) continue;

                // Merge p2 into p1
                float w1 = static_cast<float>(p1.observationCount);
                float w2 = static_cast<float>(p2.observationCount);
                float totalW = w1 + w2;

                p1.center = p1.center * (w1 / totalW) + p2.center * (w2 / totalW);
                p1.width = std::max(p1.width, p2.width);
                p1.height = std::max(p1.height, p2.height);
                p1.observationCount += p2.observationCount;

                merged[j] = true;
            }

            result.push_back(p1);
        }

        planes = result;
    }
};

// Anchor management system
class AnchorSystem {
public:
    struct Config {
        float anchorConfidenceThreshold = 0.5f;
        float anchorStabilityWindow = 1.0f;     // seconds
        int minObservationsForStable = 5;
        float maxAnchorDistance = 10.0f;        // meters
        float anchorUpdateRadius = 0.1f;        // meters
    };

    explicit AnchorSystem(const Config& config = {})
        : config_(config), nextAnchorId_(0) {}

    // Create point anchor at world position
    int64_t createPointAnchor(const Vec3& position, Timestamp timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);

        Anchor anchor;
        anchor.id = nextAnchorId_++;
        anchor.type = AnchorType::POINT;
        anchor.state = AnchorState::INITIALIZING;
        anchor.pose.position = position;
        anchor.pose.orientation = Quaternion{};
        anchor.confidence = 0.3f;
        anchor.lastSeen = timestamp;
        anchor.observationCount = 1;
        anchor.stabilityScore = 0;

        anchors_[anchor.id] = anchor;
        return anchor.id;
    }

    // Create plane anchor
    int64_t createPlaneAnchor(const Plane& plane, Timestamp timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);

        Anchor anchor;
        anchor.id = nextAnchorId_++;
        anchor.type = AnchorType::PLANE;
        anchor.state = AnchorState::INITIALIZING;
        anchor.pose.position = plane.center;

        // Orient anchor to align with plane
        // Z-axis points along plane normal
        Mat3 rot;
        rot(0, 0) = toFloat(plane.extentX.x); rot(0, 1) = toFloat(plane.extentY.x); rot(0, 2) = toFloat(plane.normal.x);
        rot(1, 0) = toFloat(plane.extentX.y); rot(1, 1) = toFloat(plane.extentY.y); rot(1, 2) = toFloat(plane.normal.y);
        rot(2, 0) = toFloat(plane.extentX.z); rot(2, 1) = toFloat(plane.extentY.z); rot(2, 2) = toFloat(plane.normal.z);

        // Convert rotation matrix to quaternion
        anchor.pose.orientation = matrixToQuaternion(rot);

        anchor.confidence = 0.4f;
        anchor.lastSeen = timestamp;
        anchor.observationCount = 1;
        anchor.planeId = plane.id;
        anchor.stabilityScore = 0;

        anchors_[anchor.id] = anchor;
        planes_[plane.id] = plane;

        return anchor.id;
    }

    // Create feature-based anchor
    int64_t createFeatureAnchor(const Vec3& position,
                               const std::vector<int64_t>& mapPointIds,
                               Timestamp timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);

        Anchor anchor;
        anchor.id = nextAnchorId_++;
        anchor.type = AnchorType::FEATURE;
        anchor.state = AnchorState::INITIALIZING;
        anchor.pose.position = position;
        anchor.pose.orientation = Quaternion{};
        anchor.confidence = 0.5f;
        anchor.lastSeen = timestamp;
        anchor.observationCount = 1;
        anchor.mapPointIds = mapPointIds;
        anchor.stabilityScore = 0;

        anchors_[anchor.id] = anchor;
        return anchor.id;
    }

    // Update anchor with new observation
    void updateAnchor(int64_t anchorId, const Pose& observedPose,
                     float confidence, Timestamp timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = anchors_.find(anchorId);
        if (it == anchors_.end()) return;

        Anchor& anchor = it->second;

        // Weighted position update
        float alpha = 0.2f * confidence;
        anchor.pose.position = anchor.pose.position * (1 - alpha) +
                              observedPose.position * alpha;

        // Orientation update via slerp
        anchor.pose.orientation = Quaternion::slerp(
            anchor.pose.orientation, observedPose.orientation, alpha);

        // Update statistics
        anchor.observationCount++;
        anchor.lastSeen = timestamp;

        // Update confidence (exponential moving average)
        anchor.confidence = 0.9f * anchor.confidence + 0.1f * confidence;

        // Update state
        if (anchor.observationCount >= config_.minObservationsForStable &&
            anchor.confidence >= config_.anchorConfidenceThreshold) {
            anchor.state = AnchorState::TRACKING;
        }

        // Update stability score
        Vec3 posDiff = observedPose.position - anchor.pose.position;
        float posError = posDiff.norm();
        anchor.stabilityScore = 0.95f * anchor.stabilityScore +
            0.05f * (1.0f - std::min(1.0f, posError / 0.01f));
    }

    // Get anchor by ID
    const Anchor* getAnchor(int64_t id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = anchors_.find(id);
        return it != anchors_.end() ? &it->second : nullptr;
    }

    // Get all active anchors
    std::vector<Anchor> getActiveAnchors() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Anchor> result;
        for (const auto& [id, anchor] : anchors_) {
            if (anchor.state == AnchorState::TRACKING ||
                anchor.state == AnchorState::LIMITED) {
                result.push_back(anchor);
            }
        }
        return result;
    }

    // Get anchors near a position
    std::vector<Anchor> getAnchorsNear(const Vec3& position, float radius) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Anchor> result;

        for (const auto& [id, anchor] : anchors_) {
            Vec3 diff = anchor.pose.position - position;
            if (diff.norm() <= radius) {
                result.push_back(anchor);
            }
        }

        return result;
    }

    // Remove anchor
    void removeAnchor(int64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        anchors_.erase(id);
    }

    // Process map points to update feature anchors
    void processMapPoints(const std::vector<MapPoint>& mapPoints,
                         Timestamp timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);

        for (auto& [id, anchor] : anchors_) {
            if (anchor.type != AnchorType::FEATURE) continue;

            // Find matching map points
            Vec3 sumPos{};
            int count = 0;

            for (auto mpId : anchor.mapPointIds) {
                for (const auto& mp : mapPoints) {
                    if (mp.id == mpId && !mp.isBad) {
                        sumPos += mp.position;
                        count++;
                        break;
                    }
                }
            }

            if (count >= 2) {
                Vec3 newPos = sumPos * (1.0f / count);
                float alpha = 0.3f;
                anchor.pose.position = anchor.pose.position * (1 - alpha) +
                                       newPos * alpha;
                anchor.lastSeen = timestamp;
                anchor.observationCount++;
                anchor.confidence = std::min(1.0f, anchor.confidence + 0.01f);
            } else {
                anchor.confidence *= 0.95f;
                if (anchor.confidence < 0.1f) {
                    anchor.state = AnchorState::LOST;
                }
            }
        }
    }

    // Update planes
    void updatePlanes(const std::vector<Plane>& detectedPlanes) {
        std::lock_guard<std::mutex> lock(mutex_);

        for (const auto& detected : detectedPlanes) {
            // Find matching existing plane
            bool found = false;
            for (auto& [id, plane] : planes_) {
                float normalDot = std::abs(plane.normal.dot(detected.normal));
                if (normalDot < 0.9f) continue;

                Vec3 diff = detected.center - plane.center;
                float dist = std::abs(diff.dot(plane.normal));
                if (dist > 0.1f) continue;

                // Update plane
                float alpha = 0.2f;
                plane.center = plane.center * (1 - alpha) + detected.center * alpha;
                plane.width = std::max(plane.width, detected.width);
                plane.height = std::max(plane.height, detected.height);
                plane.observationCount++;

                found = true;
                break;
            }

            if (!found && detected.width * detected.height >= 0.04f) {
                // Add new plane
                planes_[detected.id] = detected;
            }
        }
    }

    // Get detected planes
    std::vector<Plane> getPlanes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Plane> result;
        for (const auto& [id, plane] : planes_) {
            result.push_back(plane);
        }
        return result;
    }

private:
    Config config_;
    mutable std::mutex mutex_;

    std::unordered_map<int64_t, Anchor> anchors_;
    std::unordered_map<int64_t, Plane> planes_;
    int64_t nextAnchorId_;

    Quaternion matrixToQuaternion(const Mat3& m) {
        float trace = m(0, 0) + m(1, 1) + m(2, 2);
        Quaternion q;

        if (trace > 0) {
            float s = 0.5f / std::sqrt(trace + 1.0f);
            q.w = 0.25f / s;
            q.x = (m(2, 1) - m(1, 2)) * s;
            q.y = (m(0, 2) - m(2, 0)) * s;
            q.z = (m(1, 0) - m(0, 1)) * s;
        } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
            float s = 2.0f * std::sqrt(1.0f + m(0, 0) - m(1, 1) - m(2, 2));
            q.w = (m(2, 1) - m(1, 2)) / s;
            q.x = 0.25f * s;
            q.y = (m(0, 1) + m(1, 0)) / s;
            q.z = (m(0, 2) + m(2, 0)) / s;
        } else if (m(1, 1) > m(2, 2)) {
            float s = 2.0f * std::sqrt(1.0f + m(1, 1) - m(0, 0) - m(2, 2));
            q.w = (m(0, 2) - m(2, 0)) / s;
            q.x = (m(0, 1) + m(1, 0)) / s;
            q.y = 0.25f * s;
            q.z = (m(1, 2) + m(2, 1)) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + m(2, 2) - m(0, 0) - m(1, 1));
            q.w = (m(1, 0) - m(0, 1)) / s;
            q.x = (m(0, 2) + m(2, 0)) / s;
            q.y = (m(1, 2) + m(2, 1)) / s;
            q.z = 0.25f * s;
        }

        return q.normalized();
    }
};

}  // namespace MagicSLAM
