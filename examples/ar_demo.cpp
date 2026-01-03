/**
 * MagicSLAM AR Demo
 *
 * This example demonstrates basic usage of the MagicSLAM system
 * for AR applications on embedded platforms like Qualcomm AR1.
 */

#include <magicslam/slam_system.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>

using namespace MagicSLAM;

// Simulated sensor data generators for demo
class SensorSimulator {
public:
    SensorSimulator() : imuTimestamp_(0), frameTimestamp_(0) {}

    ImuMeasurement getIMU() {
        ImuMeasurement imu;
        imu.timestamp = imuTimestamp_;
        imuTimestamp_ += 5000000;  // 5ms = 200Hz

        // Simulate slight movement
        float t = imuTimestamp_ * 1e-9f;
        imu.acceleration = Vec3{
            toFixed(0.1f * std::sin(t)),
            toFixed(0.1f * std::cos(t)),
            toFixed(-9.81f)
        };
        imu.angularVelocity = Vec3{
            toFixed(0.01f * std::sin(t * 0.5f)),
            toFixed(0.01f * std::cos(t * 0.5f)),
            toFixed(0.005f)
        };

        return imu;
    }

    std::pair<std::vector<uint8_t>, Timestamp> getFrame() {
        frameTimestamp_ += 33333333;  // ~30 FPS

        // Generate synthetic grayscale image with features
        int width = 640;
        int height = 480;
        std::vector<uint8_t> image(width * height);

        // Create gradient background
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                image[y * width + x] = static_cast<uint8_t>(
                    128 + 50 * std::sin(x * 0.05f) * std::cos(y * 0.05f));
            }
        }

        // Add some feature points (corners)
        addCorner(image.data(), width, height, 100, 100);
        addCorner(image.data(), width, height, 540, 100);
        addCorner(image.data(), width, height, 100, 380);
        addCorner(image.data(), width, height, 540, 380);
        addCorner(image.data(), width, height, 320, 240);

        // Add random texture
        for (int i = 0; i < 50; ++i) {
            int x = rand() % (width - 20) + 10;
            int y = rand() % (height - 20) + 10;
            addCorner(image.data(), width, height, x, y);
        }

        return {image, frameTimestamp_};
    }

private:
    Timestamp imuTimestamp_;
    Timestamp frameTimestamp_;

    void addCorner(uint8_t* img, int width, int height, int cx, int cy) {
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                int x = cx + dx;
                int y = cy + dy;
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    // Create a corner pattern
                    if ((dx < 0 && dy < 0) || (dx >= 0 && dy >= 0)) {
                        img[y * width + x] = 255;
                    } else {
                        img[y * width + x] = 0;
                    }
                }
            }
        }
    }
};

int main(int argc, char* argv[]) {
    std::cout << "MagicSLAM AR Demo\n";
    std::cout << "================\n\n";

    // Configure SLAM system
    Config config;

    // Camera intrinsics (typical AR glasses camera)
    config.camera.fx = 500.0f;
    config.camera.fy = 500.0f;
    config.camera.cx = 320.0f;
    config.camera.cy = 240.0f;
    config.camera.k1 = 0.0f;  // No distortion for demo
    config.camera.k2 = 0.0f;
    config.camera.p1 = 0.0f;
    config.camera.p2 = 0.0f;
    config.camera.k3 = 0.0f;
    config.camera.width = 640;
    config.camera.height = 480;

    // Feature extraction
    config.features.numFeatures = 500;
    config.features.scaleFactor = 1.2f;
    config.features.numLevels = 4;

    // IMU parameters
    config.imu.gravity = Vec3{0, 0, -9.81f};
    config.imu.accelNoise = 0.01f;
    config.imu.gyroNoise = 0.001f;

    // Tracking
    config.tracking.minTrackedPoints = 20;

    // Power mode for AR1
    config.powerMode = Platform::PowerManager::PerformanceMode::BALANCED;

    // Create SLAM system
    SlamSystem slam(config);

    // Set tracking callback
    slam.setTrackingCallback([](TrackingState state, const Pose& pose, Timestamp ts) {
        static int frameCount = 0;
        if (++frameCount % 30 == 0) {  // Print every second
            const char* stateStr = "UNKNOWN";
            switch (state) {
                case TrackingState::NOT_INITIALIZED: stateStr = "NOT_INIT"; break;
                case TrackingState::INITIALIZING: stateStr = "INIT"; break;
                case TrackingState::TRACKING: stateStr = "TRACKING"; break;
                case TrackingState::LOST: stateStr = "LOST"; break;
                case TrackingState::RELOCALIZATION: stateStr = "RELOC"; break;
            }

            std::cout << "State: " << stateStr
                      << " | Pos: (" << toFloat(pose.position.x) << ", "
                      << toFloat(pose.position.y) << ", "
                      << toFloat(pose.position.z) << ")"
                      << " | Map: " << "N/A" << " points\n";
        }
    });

    // Set plane detection callback
    slam.setPlaneCallback([](const std::vector<Plane>& planes) {
        std::cout << "Detected " << planes.size() << " planes\n";
        for (const auto& p : planes) {
            std::cout << "  Plane " << p.id << ": "
                      << (p.isVertical ? "vertical" : "horizontal")
                      << " (" << p.width << "m x " << p.height << "m)\n";
        }
    });

    // Create sensor simulator
    SensorSimulator sensors;

    std::cout << "Starting SLAM loop...\n\n";

    // Main loop
    int imuCounter = 0;
    for (int frame = 0; frame < 300; ++frame) {  // Run for ~10 seconds

        // Process IMU at 200Hz (6-7 samples per frame at 30fps)
        for (int i = 0; i < 6; ++i) {
            auto imu = sensors.getIMU();
            slam.processIMU(imu);
            imuCounter++;
        }

        // Process camera frame at 30Hz
        auto [image, timestamp] = sensors.getFrame();
        slam.processFrame(image.data(), config.camera.width, config.camera.height, timestamp);

        // Get predicted pose for rendering
        Timestamp renderTime = timestamp + 15000000;  // +15ms for rendering latency
        Pose renderPose = slam.getPredictedPose(renderTime);

        // Simulate frame time
        std::this_thread::sleep_for(std::chrono::milliseconds(33));

        // Create an anchor after initialization
        if (frame == 100 && slam.getTrackingState() == TrackingState::TRACKING) {
            Vec3 anchorPos = slam.getCurrentPose().position;
            anchorPos.z -= 0.5f;  // 50cm below current position

            int64_t anchorId = slam.createAnchor(anchorPos);
            std::cout << "\nCreated anchor " << anchorId << " at ("
                      << toFloat(anchorPos.x) << ", "
                      << toFloat(anchorPos.y) << ", "
                      << toFloat(anchorPos.z) << ")\n\n";
        }
    }

    std::cout << "\nDemo complete!\n";
    std::cout << "Final stats:\n";
    std::cout << "  Map points: " << slam.getMapPointCount() << "\n";
    std::cout << "  Keyframes: " << slam.getKeyframeCount() << "\n";
    std::cout << "  Anchors: " << slam.getAnchors().size() << "\n";
    std::cout << "  Planes: " << slam.getPlanes().size() << "\n";
    std::cout << "  IMU samples processed: " << imuCounter << "\n";

    return 0;
}
