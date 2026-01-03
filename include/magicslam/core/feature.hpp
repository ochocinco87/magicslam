#pragma once

#include "types.hpp"
#include <array>
#include <bitset>

namespace MagicSLAM {

// ORB descriptor (256 bits = 32 bytes)
constexpr int ORB_DESCRIPTOR_SIZE = 32;

struct OrbDescriptor {
    std::array<uint8_t, ORB_DESCRIPTOR_SIZE> data;

    OrbDescriptor() { data.fill(0); }

    // Hamming distance between descriptors (optimized with popcount)
    int distance(const OrbDescriptor& other) const {
        int dist = 0;
        #if defined(__ARM_NEON) || defined(__ARM_NEON__)
        // NEON-optimized popcount for AR1
        for (int i = 0; i < ORB_DESCRIPTOR_SIZE; i += 8) {
            uint64_t v1, v2;
            std::memcpy(&v1, &data[i], 8);
            std::memcpy(&v2, &other.data[i], 8);
            dist += __builtin_popcountll(v1 ^ v2);
        }
        #else
        for (int i = 0; i < ORB_DESCRIPTOR_SIZE; ++i) {
            dist += __builtin_popcount(data[i] ^ other.data[i]);
        }
        #endif
        return dist;
    }
};

// Keypoint with ORB descriptor
struct Feature {
    Point2D position;           // Position in image
    float angle;                // Orientation
    float response;             // Keypoint response strength
    int octave;                 // Pyramid level
    OrbDescriptor descriptor;
    int64_t mapPointId;         // Associated map point (-1 if none)

    Feature() : angle(0), response(0), octave(0), mapPointId(-1) {}
};

// 3D Map point
struct MapPoint {
    int64_t id;
    Vec3 position;              // 3D position in world frame
    Vec3 normal;                // Mean viewing direction
    OrbDescriptor descriptor;   // Representative descriptor
    int observationCount;       // Number of times observed
    int matchedCount;           // Successful matches
    float minDistance;          // Min observation distance
    float maxDistance;          // Max observation distance
    bool isBad;                 // Marked for removal

    MapPoint() : id(-1), observationCount(0), matchedCount(0),
                 minDistance(0), maxDistance(std::numeric_limits<float>::max()),
                 isBad(false) {}

    // Predict scale level based on distance
    int predictScaleLevel(float distance, float scaleFactor, int numLevels) const {
        float ratio = maxDistance / distance;
        int level = static_cast<int>(std::ceil(std::log(ratio) / std::log(scaleFactor)));
        return std::max(0, std::min(level, numLevels - 1));
    }
};

// Keyframe data
struct KeyFrame {
    int64_t id;
    Timestamp timestamp;
    Pose pose;                  // Camera pose (world to camera)
    std::vector<Feature> features;
    std::vector<int64_t> mapPointIds;  // Map point IDs for each feature
    std::vector<int64_t> covisibleKeyframes;  // Connected keyframes

    // IMU preintegration from previous keyframe
    Vec3 preintegratedPosition;
    Vec3 preintegratedVelocity;
    Quaternion preintegratedRotation;

    KeyFrame() : id(-1), timestamp(0) {}

    // Check if this keyframe observes a map point
    bool observesMapPoint(int64_t mpId) const {
        for (auto id : mapPointIds) {
            if (id == mpId) return true;
        }
        return false;
    }
};

// Image pyramid for multi-scale feature detection
struct ImagePyramid {
    static constexpr int MAX_LEVELS = 8;

    struct Level {
        std::vector<uint8_t> data;
        int width;
        int height;
        float scale;
    };

    std::array<Level, MAX_LEVELS> levels;
    int numLevels;

    ImagePyramid() : numLevels(0) {}

    void build(const uint8_t* image, int width, int height,
               int nLevels, float scaleFactor) {
        numLevels = std::min(nLevels, MAX_LEVELS);

        // Level 0 is original image
        levels[0].width = width;
        levels[0].height = height;
        levels[0].scale = 1.0f;
        levels[0].data.assign(image, image + width * height);

        float scale = 1.0f;
        for (int i = 1; i < numLevels; ++i) {
            scale /= scaleFactor;
            int w = static_cast<int>(width * scale + 0.5f);
            int h = static_cast<int>(height * scale + 0.5f);
            levels[i].width = w;
            levels[i].height = h;
            levels[i].scale = scale;
            levels[i].data.resize(w * h);

            // Downsample with box filter (fast for embedded)
            downsample(levels[i - 1], levels[i], scaleFactor);
        }
    }

private:
    void downsample(const Level& src, Level& dst, float factor) {
        int srcW = src.width;
        float invFactor = 1.0f / factor;

        for (int y = 0; y < dst.height; ++y) {
            for (int x = 0; x < dst.width; ++x) {
                int srcX = static_cast<int>(x * factor);
                int srcY = static_cast<int>(y * factor);
                srcX = std::min(srcX, src.width - 1);
                srcY = std::min(srcY, src.height - 1);
                dst.data[y * dst.width + x] = src.data[srcY * srcW + srcX];
            }
        }
    }
};

}  // namespace MagicSLAM
