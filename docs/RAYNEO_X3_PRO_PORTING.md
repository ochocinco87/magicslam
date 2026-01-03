# Porting MagicSLAM to Rayneo X3 Pro

## Platform Overview

The Rayneo X3 Pro specifications:
- **Chipset**: Qualcomm Snapdragon AR1 Gen 1 (same target as MagicSLAM)
- **OS**: RayNeo AIOS (Android-based)
- **Cameras**: Sony IMX681 RGB sensor + OV spatial camera for depth/SLAM
- **SDK Options**: Unity ARDK, Android ARDK
- **6DOF Support**: Built-in SLAM tracking

## Option 1: Use RayNeo's Built-in SLAM (Recommended for Quick Start)

The X3 Pro has built-in 6DOF SLAM. You can use their SDK and skip custom SLAM:

### Setup Steps:

1. **Register as Developer**
   - Visit [open.rayneo.com](https://open.rayneo.com/)
   - Apply for developer access
   - Enable "Creator Mode" on your X3 Pro

2. **Install Development Tools**
   ```bash
   # Install Android SDK and NDK
   # Download Android Studio with NDK r25+

   # Install ADB for sideloading
   sudo apt install adb

   # Connect X3 Pro via USB
   adb devices
   ```

3. **Download RayNeo SDK**
   - Unity ARDK for Unity projects
   - Android ARDK for native Android apps

## Option 2: Deploy Custom MagicSLAM (Full Control)

If you want to use our custom SLAM implementation:

### Prerequisites

```bash
# Install Android NDK (r25 or higher for ARM64 + NEON)
export ANDROID_NDK=/path/to/android-ndk-r25c

# Install CMake for Android
sudo apt install cmake ninja-build
```

### Cross-Compile for Android ARM64

Create `android-toolchain.cmake`:

```cmake
set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_SYSTEM_VERSION 30)  # Android 11+
set(CMAKE_ANDROID_ARCH_ABI arm64-v8a)
set(CMAKE_ANDROID_NDK $ENV{ANDROID_NDK})
set(CMAKE_ANDROID_STL_TYPE c++_shared)

# Enable NEON
set(CMAKE_ANDROID_ARM_NEON ON)
```

Build MagicSLAM:

```bash
cd /home/user/magicslam
mkdir build-android && cd build-android

cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=../cmake/android-toolchain.cmake \
  -DENABLE_NEON=ON \
  -DTARGET_PLATFORM=AR1 \
  -DCMAKE_BUILD_TYPE=Release \
  -G Ninja

ninja
```

### Android App Integration

Create a JNI wrapper (`MagicSlamJNI.cpp`):

```cpp
#include <jni.h>
#include <android/log.h>
#include <magicslam/slam_system.hpp>

static MagicSLAM::SlamSystem* g_slam = nullptr;

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_example_magicslam_MagicSlamNative_create(
    JNIEnv* env, jobject thiz,
    jfloat fx, jfloat fy, jfloat cx, jfloat cy,
    jint width, jint height) {

    MagicSLAM::Config config;
    config.camera.fx = fx;
    config.camera.fy = fy;
    config.camera.cx = cx;
    config.camera.cy = cy;
    config.camera.width = width;
    config.camera.height = height;
    config.powerMode = MagicSLAM::Platform::PowerManager::PerformanceMode::BALANCED;

    g_slam = new MagicSLAM::SlamSystem(config);
    return reinterpret_cast<jlong>(g_slam);
}

JNIEXPORT void JNICALL
Java_com_example_magicslam_MagicSlamNative_processIMU(
    JNIEnv* env, jobject thiz,
    jlong timestamp, jfloat ax, jfloat ay, jfloat az,
    jfloat gx, jfloat gy, jfloat gz) {

    if (!g_slam) return;

    MagicSLAM::ImuMeasurement imu;
    imu.timestamp = static_cast<uint64_t>(timestamp);
    imu.acceleration = MagicSLAM::Vec3{ax, ay, az};
    imu.angularVelocity = MagicSLAM::Vec3{gx, gy, gz};

    g_slam->processIMU(imu);
}

JNIEXPORT void JNICALL
Java_com_example_magicslam_MagicSlamNative_processFrame(
    JNIEnv* env, jobject thiz,
    jbyteArray imageData, jint width, jint height, jlong timestamp) {

    if (!g_slam) return;

    jbyte* data = env->GetByteArrayElements(imageData, nullptr);
    g_slam->processFrame(
        reinterpret_cast<uint8_t*>(data), width, height,
        static_cast<uint64_t>(timestamp));
    env->ReleaseByteArrayElements(imageData, data, JNI_ABORT);
}

JNIEXPORT jfloatArray JNICALL
Java_com_example_magicslam_MagicSlamNative_getPredictedPose(
    JNIEnv* env, jobject thiz, jlong renderTime) {

    if (!g_slam) return nullptr;

    auto pose = g_slam->getPredictedPose(static_cast<uint64_t>(renderTime));

    jfloatArray result = env->NewFloatArray(7);
    float poseData[7] = {
        MagicSLAM::toFloat(pose.position.x),
        MagicSLAM::toFloat(pose.position.y),
        MagicSLAM::toFloat(pose.position.z),
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    };
    env->SetFloatArrayRegion(result, 0, 7, poseData);
    return result;
}

JNIEXPORT void JNICALL
Java_com_example_magicslam_MagicSlamNative_destroy(JNIEnv* env, jobject thiz) {
    delete g_slam;
    g_slam = nullptr;
}

}
```

### Kotlin/Java Wrapper

```kotlin
package com.example.magicslam

class MagicSlamNative {
    companion object {
        init {
            System.loadLibrary("magicslam_jni")
        }
    }

    external fun create(
        fx: Float, fy: Float, cx: Float, cy: Float,
        width: Int, height: Int
    ): Long

    external fun processIMU(
        timestamp: Long,
        ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float
    )

    external fun processFrame(
        imageData: ByteArray,
        width: Int, height: Int,
        timestamp: Long
    )

    external fun getPredictedPose(renderTime: Long): FloatArray

    external fun destroy()
}
```

### Access X3 Pro Sensors

```kotlin
class SlamActivity : AppCompatActivity(), SensorEventListener {
    private lateinit var sensorManager: SensorManager
    private lateinit var slam: MagicSlamNative
    private lateinit var cameraManager: CameraManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Initialize SLAM (X3 Pro camera specs)
        slam = MagicSlamNative()
        slam.create(
            fx = 500f, fy = 500f,  // Calibrate for X3 Pro
            cx = 320f, cy = 240f,
            width = 640, height = 480
        )

        // Register IMU sensors
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        sensorManager.registerListener(this, accelerometer,
            SensorManager.SENSOR_DELAY_FASTEST)  // ~200Hz
        sensorManager.registerListener(this, gyroscope,
            SensorManager.SENSOR_DELAY_FASTEST)

        // Setup camera
        setupCamera()
    }

    override fun onSensorChanged(event: SensorEvent) {
        val timestamp = event.timestamp  // nanoseconds

        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                lastAccel = event.values.clone()
            }
            Sensor.TYPE_GYROSCOPE -> {
                slam.processIMU(
                    timestamp,
                    lastAccel[0], lastAccel[1], lastAccel[2],
                    event.values[0], event.values[1], event.values[2]
                )
            }
        }
    }

    private fun onCameraFrame(image: Image, timestamp: Long) {
        // Convert to grayscale
        val yBuffer = image.planes[0].buffer
        val data = ByteArray(yBuffer.remaining())
        yBuffer.get(data)

        slam.processFrame(data, image.width, image.height, timestamp)

        // Get pose for rendering
        val renderTime = System.nanoTime() + 15_000_000L  // +15ms
        val pose = slam.getPredictedPose(renderTime)

        // pose[0-2] = position (x,y,z)
        // pose[3-6] = quaternion (w,x,y,z)
        updateARContent(pose)
    }
}
```

## Option 3: Hybrid Approach (Best of Both)

Use RayNeo's built-in SLAM for 6DOF tracking, but use MagicSLAM's:
- Head tracking prediction (for lower latency)
- Anchor system (for persistent virtual objects)
- Plane detection (for surface placement)

```kotlin
class HybridSlamService {
    private val rayneoSdk: RayneoARSession  // Their SDK
    private val magicSlam: MagicSlamNative   // Our enhancements

    fun onRayneoTrackingUpdate(pose: Pose) {
        // Feed RayNeo's pose to our head tracker
        magicSlam.updatePose(
            pose.position, pose.orientation, pose.velocity
        )
    }

    fun getPredictedPoseForRendering(): Pose {
        // Use our prediction for lower latency
        return magicSlam.getPredictedPose(renderTimestamp)
    }
}
```

## Sideloading Your App

```bash
# Build APK
./gradlew assembleRelease

# Connect X3 Pro via USB
adb devices

# Install
adb install -r app/build/outputs/apk/release/app-release.apk

# View logs
adb logcat | grep MagicSLAM
```

## Camera Calibration for X3 Pro

Run calibration to get accurate intrinsics:

```bash
# Print checkerboard pattern
# Record video with X3 Pro camera
# Extract frames and run OpenCV calibration

adb pull /sdcard/DCIM/calibration_video.mp4 .
python3 calibrate_camera.py calibration_video.mp4
# Outputs: fx, fy, cx, cy, k1, k2, p1, p2, k3
```

## Performance Tips for X3 Pro

1. **Use NEON**: Always compile with `-DENABLE_NEON=ON`
2. **Limit Features**: 300-500 features max for 30fps
3. **Power Mode**: Use BALANCED mode to avoid thermal throttling
4. **Camera Resolution**: 640x480 is optimal for SLAM
5. **IMU Rate**: 200Hz is sufficient, don't go higher

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Low FPS | Reduce `numFeatures` to 300 |
| Drift | Increase keyframe frequency |
| Jitter | Increase filter alpha values |
| Thermal throttle | Switch to LOW_POWER mode |
| Camera access denied | Enable in RayNeo AIOS settings |

## Resources

- [RayNeo Developer Portal](https://open.rayneo.com/)
- [RayNeo X3 Pro Specs](https://www.rayneo.com/products/x3-pro-ai-display-glasses)
- [Android NDK NEON Guide](https://developer.android.com/ndk/guides/cpu-arm-neon)
