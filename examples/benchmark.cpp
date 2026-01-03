/**
 * MagicSLAM Performance Benchmark
 *
 * Measures timing of critical SLAM components for AR1 optimization.
 */

#include <magicslam/slam_system.hpp>
#include <magicslam/platform/ar1_optimizations.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <iomanip>

using namespace MagicSLAM;

class Timer {
public:
    void start() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    double stop() {
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start_).count();
        times_.push_back(ms);
        return ms;
    }

    void reset() { times_.clear(); }

    double average() const {
        if (times_.empty()) return 0;
        return std::accumulate(times_.begin(), times_.end(), 0.0) / times_.size();
    }

    double min() const {
        if (times_.empty()) return 0;
        return *std::min_element(times_.begin(), times_.end());
    }

    double max() const {
        if (times_.empty()) return 0;
        return *std::max_element(times_.begin(), times_.end());
    }

private:
    std::chrono::high_resolution_clock::time_point start_;
    std::vector<double> times_;
};

void printResult(const std::string& name, const Timer& timer) {
    std::cout << std::setw(30) << std::left << name
              << std::setw(10) << std::right << std::fixed << std::setprecision(3) << timer.average()
              << std::setw(10) << timer.min()
              << std::setw(10) << timer.max()
              << " ms\n";
}

// Generate test image with features
std::vector<uint8_t> generateTestImage(int width, int height) {
    std::vector<uint8_t> image(width * height);

    // Create textured pattern
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float pattern = std::sin(x * 0.1f) * std::cos(y * 0.1f);
            pattern += std::sin(x * 0.05f + y * 0.05f);
            image[y * width + x] = static_cast<uint8_t>(128 + 64 * pattern);
        }
    }

    // Add corner features
    for (int i = 0; i < 100; ++i) {
        int cx = rand() % (width - 20) + 10;
        int cy = rand() % (height - 20) + 10;
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                if ((dx < 0) == (dy < 0)) {
                    image[(cy + dy) * width + cx + dx] = 255;
                } else {
                    image[(cy + dy) * width + cx + dx] = 0;
                }
            }
        }
    }

    return image;
}

int main() {
    std::cout << "MagicSLAM Performance Benchmark\n";
    std::cout << "================================\n\n";

    #if HAS_NEON
    std::cout << "NEON: Enabled\n";
    #else
    std::cout << "NEON: Disabled (using scalar fallbacks)\n";
    #endif

    #ifdef USE_FIXED_POINT
    std::cout << "Fixed-point: Enabled\n";
    #else
    std::cout << "Fixed-point: Disabled\n";
    #endif

    std::cout << "\n";

    const int ITERATIONS = 100;
    const int WIDTH = 640;
    const int HEIGHT = 480;

    Timer timer;

    // Generate test data
    auto testImage = generateTestImage(WIDTH, HEIGHT);
    std::vector<uint8_t> outputImage(WIDTH * HEIGHT);

    std::cout << "Running " << ITERATIONS << " iterations of each benchmark...\n\n";
    std::cout << std::setw(30) << std::left << "Operation"
              << std::setw(10) << std::right << "Avg"
              << std::setw(10) << "Min"
              << std::setw(10) << "Max"
              << "\n";
    std::cout << std::string(60, '-') << "\n";

    // Benchmark: Feature Extraction
    {
        FeatureExtractorConfig config;
        config.numFeatures = 500;
        config.numLevels = 4;
        FeatureExtractor extractor(config);

        for (int i = 0; i < ITERATIONS; ++i) {
            timer.start();
            auto features = extractor.extract(testImage.data(), WIDTH, HEIGHT);
            timer.stop();
        }
        printResult("Feature Extraction (500)", timer);
        timer.reset();
    }

    // Benchmark: ORB Descriptor Matching
    {
        std::vector<OrbDescriptor> desc1(500), desc2(500);
        for (auto& d : desc1) {
            for (auto& b : d.data) b = rand() & 0xFF;
        }
        for (auto& d : desc2) {
            for (auto& b : d.data) b = rand() & 0xFF;
        }

        for (int i = 0; i < ITERATIONS; ++i) {
            timer.start();
            int totalDist = 0;
            for (size_t j = 0; j < desc1.size(); ++j) {
                for (size_t k = 0; k < desc2.size(); ++k) {
                    totalDist += desc1[j].distance(desc2[k]);
                }
            }
            timer.stop();
            (void)totalDist;  // Prevent optimization
        }
        printResult("Descriptor Match (500x500)", timer);
        timer.reset();
    }

    // Benchmark: NEON Descriptor Distance
    {
        OrbDescriptor d1, d2;
        for (auto& b : d1.data) b = rand() & 0xFF;
        for (auto& b : d2.data) b = rand() & 0xFF;

        int dummy = 0;
        for (int i = 0; i < ITERATIONS * 10000; ++i) {
            timer.start();
            dummy += Platform::NEON::descriptorDistance(d1.data.data(), d2.data.data(), 32);
            timer.stop();
        }
        printResult("Single Hamming Dist (x10k)", timer);
        timer.reset();
        (void)dummy;
    }

    // Benchmark: Image Downsample
    {
        for (int i = 0; i < ITERATIONS; ++i) {
            timer.start();
            Platform::NEON::downsample2x(testImage.data(), outputImage.data(), WIDTH, HEIGHT);
            timer.stop();
        }
        printResult("Image Downsample 2x", timer);
        timer.reset();
    }

    // Benchmark: Gaussian Blur
    {
        for (int i = 0; i < ITERATIONS; ++i) {
            timer.start();
            Platform::NEON::gaussianBlur3x3(testImage.data(), outputImage.data(), WIDTH, HEIGHT);
            timer.stop();
        }
        printResult("Gaussian Blur 3x3", timer);
        timer.reset();
    }

    // Benchmark: IMU Integration
    {
        ImuIntegrator integrator;
        ImuMeasurement imu;
        imu.acceleration = Vec3{0.1f, 0.1f, -9.81f};
        imu.angularVelocity = Vec3{0.01f, 0.01f, 0.01f};

        for (int i = 0; i < ITERATIONS * 100; ++i) {
            imu.timestamp = i * 5000000;  // 5ms
            timer.start();
            integrator.addMeasurement(imu);
            timer.stop();
        }
        printResult("IMU Integration (x100)", timer);
        timer.reset();
    }

    // Benchmark: EKF Predict
    {
        VioEkf ekf;
        ekf.initialize(Pose{}, Vec3{}, 0);

        ImuMeasurement imu;
        imu.acceleration = Vec3{0.1f, 0.1f, -9.81f};
        imu.angularVelocity = Vec3{0.01f, 0.01f, 0.01f};

        for (int i = 0; i < ITERATIONS * 100; ++i) {
            imu.timestamp = i * 5000000;
            timer.start();
            ekf.predictIMU(imu);
            timer.stop();
        }
        printResult("EKF IMU Predict (x100)", timer);
        timer.reset();
    }

    // Benchmark: Quaternion Operations
    {
        Quaternion q1 = Quaternion::fromAxisAngle(Vec3{0, 0, 1}, 0.1f);
        Quaternion q2 = Quaternion::fromAxisAngle(Vec3{1, 0, 0}, 0.2f);
        Vec3 v{1, 2, 3};

        for (int i = 0; i < ITERATIONS * 10000; ++i) {
            timer.start();
            Quaternion q3 = q1 * q2;
            Vec3 v2 = q3.rotate(v);
            timer.stop();
            (void)v2;
        }
        printResult("Quat Mul + Rotate (x10k)", timer);
        timer.reset();
    }

    // Benchmark: Head Tracker Prediction
    {
        HeadTracker tracker;
        MotionState state;
        state.pose.position = Vec3{1, 2, 3};
        state.velocity = Vec3{0.1f, 0.1f, 0};
        state.angularVelocity = Vec3{0.01f, 0.01f, 0};
        state.timestamp = 0;

        for (int i = 0; i < ITERATIONS * 1000; ++i) {
            state.timestamp = i * 33333333;
            tracker.update(state.pose, state.velocity, state.angularVelocity, state.timestamp);

            timer.start();
            auto predicted = tracker.getPredictedPose(state.timestamp + 15000000);
            timer.stop();
            (void)predicted;
        }
        printResult("Head Prediction (x1k)", timer);
        timer.reset();
    }

    // Benchmark: Plane Detection
    {
        std::vector<Vec3> points(1000);
        for (auto& p : points) {
            p = Vec3{
                static_cast<float>(rand() % 1000) / 100.0f - 5.0f,
                static_cast<float>(rand() % 1000) / 100.0f - 5.0f,
                static_cast<float>(rand() % 100) / 100.0f
            };
        }

        PlaneDetector detector;
        for (int i = 0; i < ITERATIONS / 10; ++i) {
            timer.start();
            auto planes = detector.detectPlanes(points);
            timer.stop();
            (void)planes;
        }
        printResult("Plane Detection 1k pts", timer);
        timer.reset();
    }

    std::cout << "\n";

    // Summary
    std::cout << "Performance Summary for AR1 Target:\n";
    std::cout << "-----------------------------------\n";

    // Calculate frame budget
    float frameTime = 33.3f;  // 30 FPS target
    std::cout << "Target frame time: " << frameTime << " ms (30 FPS)\n";

    std::cout << "\nRecommended settings for AR1:\n";
    std::cout << "  - Features: 300-500\n";
    std::cout << "  - Pyramid levels: 4\n";
    std::cout << "  - IMU rate: 200 Hz\n";
    std::cout << "  - Use NEON optimizations\n";
    std::cout << "  - Consider fixed-point for DSP offload\n";

    return 0;
}
