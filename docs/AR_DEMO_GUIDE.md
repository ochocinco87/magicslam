# MagicSLAM AR Demo Guide

This guide explains how to build and run the AR demo that demonstrates stable virtual object placement in physical space.

## What the Demo Shows

### 1. AR Demo Activity
- **Tap anywhere** to place virtual objects (cubes, spheres, pyramids)
- Objects are **anchored to physical space** using SLAM
- As you **move around**, objects stay in their real-world positions
- Different colored objects help visualize multiple placements

### 2. Stability Test Activity
- Places a **3x3 grid of test objects** in front of you
- **Measures drift** in real-time (how much objects move from initial position)
- **Color-coded feedback**:
  - 🟢 Green: < 5mm drift (excellent)
  - 🟡 Yellow: < 20mm drift (good)
  - 🟠 Orange: < 50mm drift (acceptable)
  - 🔴 Red: > 50mm drift (poor)
- Walk around the grid and watch objects **stay anchored**

## Building for Rayneo X3 Pro

### Prerequisites

```bash
# 1. Install Android Studio with NDK
# - Download from developer.android.com
# - Install NDK 25.2.9519653 or higher
# - Install CMake 3.22.1

# 2. Set environment variables
export ANDROID_HOME=$HOME/Android/Sdk
export ANDROID_NDK=$ANDROID_HOME/ndk/25.2.9519653
export PATH=$PATH:$ANDROID_HOME/platform-tools
```

### Build Steps

```bash
cd /home/user/magicslam/android

# Build debug APK
./gradlew assembleDebug

# Or build release APK
./gradlew assembleRelease

# APK location
ls -la app/build/outputs/apk/debug/app-debug.apk
```

### Install on Rayneo X3 Pro

```bash
# 1. Enable Developer Mode on X3 Pro
#    - Go to Settings > About > Tap "Build Number" 7 times
#    - Enable USB debugging

# 2. Connect via USB
adb devices
# Should show your device

# 3. Install APK
adb install -r app/build/outputs/apk/debug/app-debug.apk

# 4. Launch the app
adb shell am start -n com.magicslam.demo/.ARDemoActivity

# 5. View logs
adb logcat | grep -E "(MagicSLAM|ARDemo|StabilityTest)"
```

## Running the Stability Test

### Step-by-Step Instructions

1. **Launch the app** on your X3 Pro
2. **Move around slowly** to initialize SLAM tracking
   - The status will change from "INITIALIZING" to "TRACKING"
   - Map points count should increase
3. **Press "Place Test Grid"** when ready
   - A 3x3 grid of cubes appears 1.5m in front of you
4. **Walk around the grid**
   - Move left, right, forward, backward
   - Walk in circles around the objects
   - The cubes should **stay in place**
5. **Monitor drift values**
   - Current drift shows real-time measurement
   - Average and max drift track overall stability
   - Color coding shows tracking quality

### Expected Results

| Metric | Excellent | Good | Acceptable |
|--------|-----------|------|------------|
| Max Drift | < 5mm | < 20mm | < 50mm |
| Avg Drift | < 2mm | < 10mm | < 25mm |
| After 60s | < 10mm | < 30mm | < 100mm |

### Troubleshooting

| Issue | Solution |
|-------|----------|
| Objects drift slowly | Ensure good lighting, textured surfaces |
| Objects jump suddenly | Tracking was lost, move more slowly |
| Low map point count | Add more visual features (posters, objects) |
| High drift indoors | Add more lighting, avoid blank walls |

## Camera Calibration

For best results, calibrate the camera for your specific X3 Pro unit:

```bash
# 1. Record checkerboard video
adb shell am start -n com.android.camera/.Camera
# Record a 9x6 checkerboard from multiple angles

# 2. Pull video
adb pull /sdcard/DCIM/Camera/calibration.mp4

# 3. Run calibration (OpenCV)
python3 calibrate_camera.py calibration.mp4
# Outputs: fx, fy, cx, cy, k1, k2, p1, p2, k3

# 4. Update camera parameters in ARDemoActivity.kt
const val CAMERA_FX = <your_fx>
const val CAMERA_FY = <your_fy>
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                       ARDemoActivity                         │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │
│  │ Camera2 API │───▶│ MagicSlam   │───▶│  Virtual        │ │
│  │ (30 FPS)    │    │ (processFrame)   │  Objects Map    │ │
│  └─────────────┘    └─────────────┘    └─────────────────┘ │
│                            │                     │          │
│  ┌─────────────┐           │                     │          │
│  │ SensorManager│──────────┘                     │          │
│  │ (200 Hz IMU)│                                 │          │
│  └─────────────┘                                 ▼          │
│                     ┌─────────────────────────────────────┐ │
│                     │           ARRenderer                │ │
│                     │  • Pose from getPredictedPose()    │ │
│                     │  • Anchor poses from getAnchorPose()│ │
│                     │  • OpenGL ES 2.0 rendering         │ │
│                     └─────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Key Features Demonstrated

### 1. Stable 6DOF Tracking
- Full position (X, Y, Z) and orientation (pitch, yaw, roll)
- IMU prediction at 200Hz for smooth motion
- Visual updates at 30Hz for accuracy

### 2. Persistent Anchors
- Objects stay where you put them
- Survive walking around and returning
- Confidence-based transparency shows tracking quality

### 3. Low Latency Rendering
- 15ms prediction lookahead compensates for display latency
- Smooth head tracking without swim/lag
- Async Time Warp ready

### 4. Plane Detection
- Automatically detects horizontal and vertical surfaces
- Shows plane grids for surface awareness
- Enables placing objects "on" surfaces

## Performance Metrics

Target performance on Qualcomm AR1:

| Metric | Target | Notes |
|--------|--------|-------|
| Frame rate | 30 FPS | Camera-bound |
| IMU rate | 200 Hz | Sensor-bound |
| Latency | < 20ms | Motion to photon |
| Features | 300-500 | Per frame |
| Map points | 1000+ | After 30s |
| Memory | < 100MB | Peak usage |

## Files Structure

```
android/
├── app/src/main/
│   ├── java/com/magicslam/
│   │   ├── MagicSlamNative.kt      # Kotlin wrapper
│   │   └── demo/
│   │       ├── ARDemoActivity.kt    # Main AR demo
│   │       ├── ARRenderer.kt        # OpenGL renderer
│   │       └── StabilityTestActivity.kt  # Drift test
│   ├── res/
│   │   └── values/
│   │       ├── strings.xml
│   │       └── themes.xml
│   └── AndroidManifest.xml
├── jni/
│   ├── CMakeLists.txt              # Native build
│   └── magicslam_jni.cpp           # JNI bindings
├── build.gradle.kts
└── settings.gradle.kts
```

## Next Steps

1. **Add persistent storage** - Save/load anchors across sessions
2. **Add cloud anchors** - Share anchors between devices
3. **Add mesh reconstruction** - Real-time 3D mesh of environment
4. **Add occlusion** - Virtual objects hidden behind real objects
5. **Add gesture recognition** - Hand tracking for interaction
