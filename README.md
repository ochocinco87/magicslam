# MagicSLAM - AR Glasses SLAM System

A lightweight, high-performance Visual-Inertial SLAM system designed for AR glasses
running on embedded chipsets like Qualcomm AR1.

## Features

- **6DOF Tracking**: Full position and orientation tracking
- **Visual-Inertial Fusion**: Tightly-coupled IMU and camera integration
- **Head Tracking Prediction**: Low-latency pose prediction for AR rendering
- **Virtual Object Anchoring**: Stable placement and persistence of virtual objects
- **Embedded Optimized**: NEON SIMD, fixed-point math, memory-efficient algorithms

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        MagicSLAM Pipeline                        │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────┐    ┌─────────────┐    ┌──────────────────────────┐ │
│  │   IMU   │───▶│ IMU Integr. │───▶│                          │ │
│  │ @200Hz  │    │ + Predict   │    │   Extended Kalman Filter │ │
│  └─────────┘    └─────────────┘    │   (Visual-Inertial       │ │
│                                     │    Fusion)               │ │
│  ┌─────────┐    ┌─────────────┐    │                          │ │
│  │ Camera  │───▶│ ORB Feature │───▶│                          │ │
│  │ @30Hz   │    │ Extraction  │    └────────────┬─────────────┘ │
│  └─────────┘    └─────────────┘                 │               │
│                                                  ▼               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    Map Management                         │   │
│  │  • Keyframe Selection    • Local Bundle Adjustment       │   │
│  │  • Map Point Culling     • Loop Closure Detection        │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                  │               │
│                                                  ▼               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              Head Tracking Prediction                     │   │
│  │  • Velocity-based prediction   • Jitter filtering        │   │
│  │  • Latency compensation        • Smooth interpolation    │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                  │               │
│                                                  ▼               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              Virtual Object Anchoring                     │   │
│  │  • Anchor creation/update      • Persistence             │   │
│  │  • Plane detection             • Occlusion handling      │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Requirements

- C++17 compiler
- OpenCV 4.x (for feature extraction)
- Eigen3 (for linear algebra)
- Optional: NEON-enabled ARM toolchain for AR1 deployment

## Building

```bash
mkdir build && cd build
cmake .. -DENABLE_NEON=ON -DTARGET_PLATFORM=AR1
make -j4
```

## Usage

```cpp
#include "magicslam/slam_system.hpp"

MagicSLAM::Config config;
config.camera.fx = 500.0f;
config.camera.fy = 500.0f;
// ... configure other parameters

MagicSLAM::SlamSystem slam(config);

// Processing loop
while (running) {
    slam.processIMU(imu_data);

    if (new_frame_available) {
        slam.processFrame(image, timestamp);
    }

    // Get predicted pose for rendering (compensates for latency)
    auto pose = slam.getPredictedPose(render_timestamp);

    // Get anchored objects
    auto anchors = slam.getAnchors();
}
```

## License

MIT License
