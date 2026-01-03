#pragma once

#include "../core/types.hpp"
#include "../core/feature.hpp"
#include <algorithm>
#include <cstring>

namespace MagicSLAM {

// ORB pattern for descriptor computation (pre-computed)
// Simplified 256-bit pattern optimized for embedded
static const int ORB_PATTERN[256 * 4] = {
    8,-3, 9,5, 4,2, 7,-12, -11,9, -8,2, 7,-12, 12,-13, 2,-13, 2,12, 1,-7, 1,2,
    -2,11, -2,-10, -13,12, -8,10, -1,-11, 7,-4, -13,-3, 0,3, 8,-6, 4,-7,
    // ... (truncated for brevity - full pattern would be 1024 integers)
    // These are relative pixel offsets for ORB bit tests
    -1,0, 0,1, 1,0, 0,-1, -1,-1, 1,1, -1,1, 1,-1,
    -2,0, 0,2, 2,0, 0,-2, -2,-2, 2,2, -2,2, 2,-2,
    -3,0, 0,3, 3,0, 0,-3, -3,-1, 3,1, -1,3, 1,-3,
    -3,-3, 3,3, -3,3, 3,-3, -4,0, 0,4, 4,0, 0,-4,
    -4,-1, 4,1, -1,4, 1,-4, -4,-2, 4,2, -2,4, 2,-4,
    -5,0, 0,5, 5,0, 0,-5, -5,-1, 5,1, -1,5, 1,-5,
    -5,-2, 5,2, -2,5, 2,-5, -5,-3, 5,3, -3,5, 3,-5,
    -6,0, 0,6, 6,0, 0,-6, -6,-1, 6,1, -1,6, 1,-6,
    -6,-2, 6,2, -2,6, 2,-6, -6,-3, 6,3, -3,6, 3,-6,
    -7,0, 0,7, 7,0, 0,-7, -7,-1, 7,1, -1,7, 1,-7,
    -7,-2, 7,2, -2,7, 2,-7, -7,-3, 7,3, -3,7, 3,-7,
    -8,0, 0,8, 8,0, 0,-8, -8,-1, 8,1, -1,8, 1,-8,
    // Remaining pattern entries...
};

// FAST corner pattern (16 pixels in Bresenham circle)
static const int FAST_PATTERN[16][2] = {
    {0, -3}, {1, -3}, {2, -2}, {3, -1},
    {3, 0}, {3, 1}, {2, 2}, {1, 3},
    {0, 3}, {-1, 3}, {-2, 2}, {-3, 1},
    {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3}
};

struct FeatureExtractorConfig {
    int numFeatures = 1000;         // Target features per frame
    float scaleFactor = 1.2f;       // Pyramid scale factor
    int numLevels = 8;              // Pyramid levels
    int fastThreshold = 20;         // FAST threshold
    int edgeMargin = 19;            // Edge margin for descriptors
    int patchSize = 31;             // ORB patch size
    bool nonMaxSuppression = true;  // Apply non-max suppression
};

class FeatureExtractor {
public:
    explicit FeatureExtractor(const FeatureExtractorConfig& config = {})
        : config_(config) {
        // Precompute scale factors
        scaleFactors_.resize(config_.numLevels);
        invScaleFactors_.resize(config_.numLevels);
        levelSigma2_.resize(config_.numLevels);

        scaleFactors_[0] = 1.0f;
        levelSigma2_[0] = 1.0f;
        for (int i = 1; i < config_.numLevels; ++i) {
            scaleFactors_[i] = scaleFactors_[i-1] * config_.scaleFactor;
            levelSigma2_[i] = scaleFactors_[i] * scaleFactors_[i];
        }
        for (int i = 0; i < config_.numLevels; ++i) {
            invScaleFactors_[i] = 1.0f / scaleFactors_[i];
        }

        // Compute features per level (more at higher resolution)
        computeFeaturesPerLevel();
    }

    // Extract features from grayscale image
    std::vector<Feature> extract(const uint8_t* image, int width, int height) {
        // Build image pyramid
        pyramid_.build(image, width, height, config_.numLevels, config_.scaleFactor);

        std::vector<Feature> allFeatures;
        allFeatures.reserve(config_.numFeatures);

        // Extract features at each level
        for (int level = 0; level < config_.numLevels; ++level) {
            auto levelFeatures = extractLevel(level);

            // Scale coordinates to level 0
            for (auto& f : levelFeatures) {
                f.position.x *= scaleFactors_[level];
                f.position.y *= scaleFactors_[level];
                f.octave = level;
            }

            allFeatures.insert(allFeatures.end(),
                             levelFeatures.begin(), levelFeatures.end());
        }

        // Sort by response and keep top N
        std::sort(allFeatures.begin(), allFeatures.end(),
                  [](const Feature& a, const Feature& b) {
                      return a.response > b.response;
                  });

        if (allFeatures.size() > static_cast<size_t>(config_.numFeatures)) {
            allFeatures.resize(config_.numFeatures);
        }

        return allFeatures;
    }

    const std::vector<float>& getScaleFactors() const { return scaleFactors_; }
    const std::vector<float>& getInvScaleFactors() const { return invScaleFactors_; }

private:
    FeatureExtractorConfig config_;
    ImagePyramid pyramid_;
    std::vector<float> scaleFactors_;
    std::vector<float> invScaleFactors_;
    std::vector<float> levelSigma2_;
    std::vector<int> featuresPerLevel_;

    void computeFeaturesPerLevel() {
        featuresPerLevel_.resize(config_.numLevels);
        float factor = 1.0f / config_.scaleFactor;
        float nDesiredFeaturesPerScale = config_.numFeatures *
            (1 - factor) / (1 - std::pow(factor, config_.numLevels));

        int sumFeatures = 0;
        for (int level = 0; level < config_.numLevels - 1; ++level) {
            featuresPerLevel_[level] = static_cast<int>(nDesiredFeaturesPerScale);
            sumFeatures += featuresPerLevel_[level];
            nDesiredFeaturesPerScale *= factor;
        }
        featuresPerLevel_[config_.numLevels - 1] = config_.numFeatures - sumFeatures;
    }

    std::vector<Feature> extractLevel(int level) {
        const auto& img = pyramid_.levels[level];
        std::vector<Feature> features;

        // FAST corner detection
        auto corners = detectFAST(img.data.data(), img.width, img.height);

        // Non-max suppression
        if (config_.nonMaxSuppression) {
            corners = nonMaxSuppression(corners, img.width, img.height);
        }

        // Limit features per level
        if (corners.size() > static_cast<size_t>(featuresPerLevel_[level])) {
            std::partial_sort(corners.begin(),
                            corners.begin() + featuresPerLevel_[level],
                            corners.end(),
                            [](const Feature& a, const Feature& b) {
                                return a.response > b.response;
                            });
            corners.resize(featuresPerLevel_[level]);
        }

        // Compute orientation and descriptors
        for (auto& corner : corners) {
            if (corner.position.x >= config_.edgeMargin &&
                corner.position.x < img.width - config_.edgeMargin &&
                corner.position.y >= config_.edgeMargin &&
                corner.position.y < img.height - config_.edgeMargin) {

                corner.angle = computeOrientation(img.data.data(), img.width,
                    static_cast<int>(corner.position.x),
                    static_cast<int>(corner.position.y));

                computeDescriptor(img.data.data(), img.width,
                    static_cast<int>(corner.position.x),
                    static_cast<int>(corner.position.y),
                    corner.angle, corner.descriptor);

                features.push_back(corner);
            }
        }

        return features;
    }

    // FAST-9 corner detector (optimized)
    std::vector<Feature> detectFAST(const uint8_t* img, int width, int height) {
        std::vector<Feature> corners;
        const int threshold = config_.fastThreshold;
        const int margin = 3;

        for (int y = margin; y < height - margin; ++y) {
            for (int x = margin; x < width - margin; ++x) {
                int center = img[y * width + x];

                // Quick check: test pixels at 0, 4, 8, 12 positions
                int p0 = img[(y + FAST_PATTERN[0][1]) * width + x + FAST_PATTERN[0][0]];
                int p4 = img[(y + FAST_PATTERN[4][1]) * width + x + FAST_PATTERN[4][0]];
                int p8 = img[(y + FAST_PATTERN[8][1]) * width + x + FAST_PATTERN[8][0]];
                int p12 = img[(y + FAST_PATTERN[12][1]) * width + x + FAST_PATTERN[12][0]];

                int countBright = (p0 > center + threshold) + (p4 > center + threshold) +
                                 (p8 > center + threshold) + (p12 > center + threshold);
                int countDark = (p0 < center - threshold) + (p4 < center - threshold) +
                               (p8 < center - threshold) + (p12 < center - threshold);

                if (countBright < 3 && countDark < 3) continue;

                // Full FAST-9 check
                int bright = 0, dark = 0;
                int maxBright = 0, maxDark = 0;
                int score = 0;

                for (int i = 0; i < 25; ++i) {  // Check with wrap-around
                    int idx = i % 16;
                    int px = x + FAST_PATTERN[idx][0];
                    int py = y + FAST_PATTERN[idx][1];
                    int val = img[py * width + px];

                    if (val > center + threshold) {
                        bright++;
                        dark = 0;
                        score += val - center;
                        if (bright > maxBright) maxBright = bright;
                    } else if (val < center - threshold) {
                        dark++;
                        bright = 0;
                        score += center - val;
                        if (dark > maxDark) maxDark = dark;
                    } else {
                        bright = 0;
                        dark = 0;
                    }
                }

                if (maxBright >= 9 || maxDark >= 9) {
                    Feature f;
                    f.position = Point2D(static_cast<float>(x), static_cast<float>(y));
                    f.response = static_cast<float>(score);
                    corners.push_back(f);
                }
            }
        }

        return corners;
    }

    // Non-maximum suppression in 3x3 neighborhood
    std::vector<Feature> nonMaxSuppression(const std::vector<Feature>& corners,
                                           int width, int height) {
        if (corners.empty()) return {};

        // Create response grid
        std::vector<float> grid(width * height, 0);
        for (const auto& c : corners) {
            int idx = static_cast<int>(c.position.y) * width +
                     static_cast<int>(c.position.x);
            if (idx >= 0 && idx < width * height) {
                grid[idx] = c.response;
            }
        }

        std::vector<Feature> result;
        for (const auto& c : corners) {
            int x = static_cast<int>(c.position.x);
            int y = static_cast<int>(c.position.y);
            float response = c.response;
            bool isMax = true;

            for (int dy = -1; dy <= 1 && isMax; ++dy) {
                for (int dx = -1; dx <= 1 && isMax; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        if (grid[ny * width + nx] > response) {
                            isMax = false;
                        }
                    }
                }
            }

            if (isMax) {
                result.push_back(c);
            }
        }

        return result;
    }

    // Compute feature orientation using intensity centroid
    float computeOrientation(const uint8_t* img, int stride, int x, int y) {
        constexpr int HALF_PATCH = 15;
        int m01 = 0, m10 = 0;

        for (int v = -HALF_PATCH; v <= HALF_PATCH; ++v) {
            for (int u = -HALF_PATCH; u <= HALF_PATCH; ++u) {
                if (u * u + v * v <= HALF_PATCH * HALF_PATCH) {
                    int val = img[(y + v) * stride + x + u];
                    m10 += u * val;
                    m01 += v * val;
                }
            }
        }

        return std::atan2(static_cast<float>(m01), static_cast<float>(m10));
    }

    // Compute ORB descriptor with rotation
    void computeDescriptor(const uint8_t* img, int stride, int x, int y,
                          float angle, OrbDescriptor& desc) {
        float cosA = std::cos(angle);
        float sinA = std::sin(angle);

        desc.data.fill(0);

        for (int i = 0; i < 256; ++i) {
            // Get pattern points (simplified - would use full pattern)
            int patIdx = (i * 4) % (sizeof(ORB_PATTERN) / sizeof(int));
            int dx1 = ORB_PATTERN[patIdx];
            int dy1 = ORB_PATTERN[patIdx + 1];
            int dx2 = ORB_PATTERN[patIdx + 2];
            int dy2 = ORB_PATTERN[patIdx + 3];

            // Rotate pattern
            int rx1 = static_cast<int>(cosA * dx1 - sinA * dy1);
            int ry1 = static_cast<int>(sinA * dx1 + cosA * dy1);
            int rx2 = static_cast<int>(cosA * dx2 - sinA * dy2);
            int ry2 = static_cast<int>(sinA * dx2 + cosA * dy2);

            // Compare pixels
            int val1 = img[(y + ry1) * stride + x + rx1];
            int val2 = img[(y + ry2) * stride + x + rx2];

            if (val1 < val2) {
                desc.data[i / 8] |= (1 << (i % 8));
            }
        }
    }
};

// Feature matcher using ORB descriptors
class FeatureMatcher {
public:
    struct Match {
        int queryIdx;
        int trainIdx;
        int distance;
    };

    FeatureMatcher(int maxDistance = 50, float ratioThreshold = 0.7f)
        : maxDistance_(maxDistance), ratioThreshold_(ratioThreshold) {}

    // Match features using brute force with ratio test
    std::vector<Match> match(const std::vector<Feature>& query,
                             const std::vector<Feature>& train) {
        std::vector<Match> matches;
        matches.reserve(query.size());

        for (size_t i = 0; i < query.size(); ++i) {
            int bestDist = 256;
            int secondBestDist = 256;
            int bestIdx = -1;

            for (size_t j = 0; j < train.size(); ++j) {
                int dist = query[i].descriptor.distance(train[j].descriptor);

                if (dist < bestDist) {
                    secondBestDist = bestDist;
                    bestDist = dist;
                    bestIdx = static_cast<int>(j);
                } else if (dist < secondBestDist) {
                    secondBestDist = dist;
                }
            }

            // Apply distance and ratio thresholds
            if (bestIdx >= 0 && bestDist <= maxDistance_) {
                if (secondBestDist == 256 ||
                    static_cast<float>(bestDist) < ratioThreshold_ * secondBestDist) {
                    matches.push_back({static_cast<int>(i), bestIdx, bestDist});
                }
            }
        }

        return matches;
    }

    // Match features to map points (with scale and viewing angle constraints)
    std::vector<Match> matchToMap(const std::vector<Feature>& features,
                                  const std::vector<MapPoint>& mapPoints,
                                  const Pose& cameraPose,
                                  const CameraIntrinsics& camera,
                                  const std::vector<float>& scaleFactors) {
        std::vector<Match> matches;

        for (size_t i = 0; i < features.size(); ++i) {
            const auto& f = features[i];
            int bestDist = maxDistance_;
            int bestIdx = -1;

            for (size_t j = 0; j < mapPoints.size(); ++j) {
                const auto& mp = mapPoints[j];
                if (mp.isBad) continue;

                // Transform to camera frame
                Vec3 pCam = cameraPose.inverseTransformPoint(mp.position);
                if (pCam.z <= 0) continue;

                // Project to image
                Point2D proj = camera.project(pCam);
                if (!camera.isInImage(proj, 10)) continue;

                // Check distance to feature
                float dx = proj.x - f.position.x;
                float dy = proj.y - f.position.y;
                float dist2D = std::sqrt(dx * dx + dy * dy);

                // Search radius based on scale
                float radius = 15.0f * scaleFactors[f.octave];
                if (dist2D > radius) continue;

                // Descriptor distance
                int descDist = f.descriptor.distance(mp.descriptor);
                if (descDist < bestDist) {
                    bestDist = descDist;
                    bestIdx = static_cast<int>(j);
                }
            }

            if (bestIdx >= 0) {
                matches.push_back({static_cast<int>(i), bestIdx, bestDist});
            }
        }

        return matches;
    }

private:
    int maxDistance_;
    float ratioThreshold_;
};

}  // namespace MagicSLAM
