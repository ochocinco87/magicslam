/**
 * Unit tests for feature extraction
 */

#include <magicslam/vision/feature_extractor.hpp>
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

using namespace MagicSLAM;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    test_##name(); \
    std::cout << "PASSED\n"; \
} while(0)

// Generate test image with known features
std::vector<uint8_t> generateTestImage(int width, int height) {
    std::vector<uint8_t> image(width * height, 128);

    // Add corners at known locations
    auto addCorner = [&](int cx, int cy) {
        for (int dy = -5; dy <= 5; ++dy) {
            for (int dx = -5; dx <= 5; ++dx) {
                int x = cx + dx;
                int y = cy + dy;
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    if ((dx < 0) == (dy < 0)) {
                        image[y * width + x] = 255;
                    } else {
                        image[y * width + x] = 0;
                    }
                }
            }
        }
    };

    // Add corners in a grid pattern
    for (int y = 50; y < height - 50; y += 100) {
        for (int x = 50; x < width - 50; x += 100) {
            addCorner(x, y);
        }
    }

    return image;
}

TEST(descriptor_distance) {
    OrbDescriptor d1, d2;

    // Same descriptor should have distance 0
    for (int i = 0; i < 32; ++i) {
        d1.data[i] = 0xAA;
        d2.data[i] = 0xAA;
    }
    assert(d1.distance(d2) == 0);

    // All bits different should have distance 256
    for (int i = 0; i < 32; ++i) {
        d2.data[i] = 0x55;  // Complement of 0xAA
    }
    assert(d1.distance(d2) == 256);

    // Half bits different
    for (int i = 0; i < 16; ++i) {
        d2.data[i] = 0xAA;
    }
    assert(d1.distance(d2) == 128);
}

TEST(feature_extraction_basic) {
    int width = 640;
    int height = 480;
    auto image = generateTestImage(width, height);

    FeatureExtractorConfig config;
    config.numFeatures = 100;
    config.numLevels = 4;
    config.fastThreshold = 10;

    FeatureExtractor extractor(config);
    auto features = extractor.extract(image.data(), width, height);

    // Should detect some features
    assert(features.size() > 0);
    std::cout << "(found " << features.size() << " features) ";

    // Features should be within image bounds
    for (const auto& f : features) {
        assert(f.position.x >= 0 && f.position.x < width);
        assert(f.position.y >= 0 && f.position.y < height);
    }
}

TEST(feature_extraction_pyramid) {
    int width = 640;
    int height = 480;
    auto image = generateTestImage(width, height);

    FeatureExtractorConfig config;
    config.numFeatures = 200;
    config.numLevels = 4;
    config.scaleFactor = 1.2f;

    FeatureExtractor extractor(config);
    auto features = extractor.extract(image.data(), width, height);

    // Check that features are extracted at multiple octaves
    std::vector<int> octaveCounts(config.numLevels, 0);
    for (const auto& f : features) {
        assert(f.octave >= 0 && f.octave < config.numLevels);
        octaveCounts[f.octave]++;
    }

    // Should have features at multiple levels
    int nonEmptyLevels = 0;
    for (int count : octaveCounts) {
        if (count > 0) nonEmptyLevels++;
    }
    assert(nonEmptyLevels >= 2);
}

TEST(feature_matching) {
    // Create two sets of similar features
    std::vector<Feature> features1, features2;

    for (int i = 0; i < 50; ++i) {
        Feature f1, f2;
        f1.position = Point2D(i * 10.0f, i * 5.0f);
        f2.position = Point2D(i * 10.0f + 2, i * 5.0f + 2);

        // Similar descriptors (small difference)
        for (int j = 0; j < 32; ++j) {
            f1.descriptor.data[j] = (i * 17 + j) & 0xFF;
            f2.descriptor.data[j] = f1.descriptor.data[j] ^ (j == 0 ? 0x01 : 0);  // 1-bit difference
        }

        features1.push_back(f1);
        features2.push_back(f2);
    }

    // Add some unmatched features
    for (int i = 0; i < 20; ++i) {
        Feature f;
        f.position = Point2D(500 + i * 5.0f, 300 + i * 3.0f);
        for (int j = 0; j < 32; ++j) {
            f.descriptor.data[j] = rand() & 0xFF;
        }
        features2.push_back(f);
    }

    FeatureMatcher matcher(50, 0.8f);
    auto matches = matcher.match(features1, features2);

    // Should match most of the similar features
    assert(matches.size() >= 40);

    // Matches should be correct
    for (const auto& m : matches) {
        if (m.queryIdx < 50 && m.trainIdx < 50) {
            // Corresponding features should match
            assert(m.queryIdx == m.trainIdx);
        }
    }
}

TEST(image_pyramid) {
    int width = 640;
    int height = 480;
    auto image = generateTestImage(width, height);

    ImagePyramid pyramid;
    pyramid.build(image.data(), width, height, 4, 1.2f);

    assert(pyramid.numLevels == 4);

    // Level 0 should be original size
    assert(pyramid.levels[0].width == width);
    assert(pyramid.levels[0].height == height);

    // Each level should be smaller
    for (int i = 1; i < 4; ++i) {
        assert(pyramid.levels[i].width < pyramid.levels[i-1].width);
        assert(pyramid.levels[i].height < pyramid.levels[i-1].height);
        assert(pyramid.levels[i].scale < pyramid.levels[i-1].scale);
    }
}

TEST(feature_response) {
    int width = 640;
    int height = 480;
    auto image = generateTestImage(width, height);

    FeatureExtractorConfig config;
    config.numFeatures = 100;

    FeatureExtractor extractor(config);
    auto features = extractor.extract(image.data(), width, height);

    // Features should have positive response
    for (const auto& f : features) {
        assert(f.response > 0);
    }

    // Features should be sorted by response (descending)
    for (size_t i = 1; i < features.size(); ++i) {
        assert(features[i].response <= features[i-1].response);
    }
}

TEST(empty_image) {
    int width = 640;
    int height = 480;
    std::vector<uint8_t> image(width * height, 128);  // Uniform gray

    FeatureExtractorConfig config;
    config.numFeatures = 100;

    FeatureExtractor extractor(config);
    auto features = extractor.extract(image.data(), width, height);

    // Uniform image should have few/no features
    assert(features.size() < 10);
}

TEST(scale_factors) {
    FeatureExtractorConfig config;
    config.numLevels = 4;
    config.scaleFactor = 1.2f;

    FeatureExtractor extractor(config);

    const auto& scales = extractor.getScaleFactors();
    assert(scales.size() == 4);
    assert(std::abs(scales[0] - 1.0f) < 0.001f);
    assert(std::abs(scales[1] - 1.2f) < 0.001f);
    assert(std::abs(scales[2] - 1.44f) < 0.01f);

    const auto& invScales = extractor.getInvScaleFactors();
    assert(invScales.size() == 4);
    assert(std::abs(invScales[0] * scales[0] - 1.0f) < 0.001f);
    assert(std::abs(invScales[1] * scales[1] - 1.0f) < 0.001f);
}

int main() {
    std::cout << "Running feature extraction tests...\n\n";

    RUN_TEST(descriptor_distance);
    RUN_TEST(feature_extraction_basic);
    RUN_TEST(feature_extraction_pyramid);
    RUN_TEST(feature_matching);
    RUN_TEST(image_pyramid);
    RUN_TEST(feature_response);
    RUN_TEST(empty_image);
    RUN_TEST(scale_factors);

    std::cout << "\nAll tests passed!\n";
    return 0;
}
