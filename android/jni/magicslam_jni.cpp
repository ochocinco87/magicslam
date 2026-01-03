/**
 * MagicSLAM JNI Bindings for Android/Rayneo X3 Pro
 *
 * Provides Java/Kotlin interface to the native SLAM system.
 */

#include <jni.h>
#include <android/log.h>
#include <magicslam/slam_system.hpp>
#include <memory>
#include <mutex>

#define LOG_TAG "MagicSLAM"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

using namespace MagicSLAM;

// Global SLAM instance (one per process)
static std::unique_ptr<SlamSystem> g_slam;
static std::mutex g_mutex;

// Callback references
static JavaVM* g_jvm = nullptr;
static jobject g_callbackObj = nullptr;
static jmethodID g_onTrackingStateChanged = nullptr;
static jmethodID g_onPlaneDetected = nullptr;

// Helper to get JNIEnv for callbacks
static JNIEnv* getEnv() {
    JNIEnv* env = nullptr;
    if (g_jvm) {
        g_jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    }
    return env;
}

extern "C" {

JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM* vm, void* reserved) {
    g_jvm = vm;
    LOGI("MagicSLAM JNI loaded");
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL
JNI_OnUnload(JavaVM* vm, void* reserved) {
    g_slam.reset();
    g_jvm = nullptr;
    LOGI("MagicSLAM JNI unloaded");
}

/**
 * Create SLAM system with camera intrinsics
 */
JNIEXPORT jboolean JNICALL
Java_com_magicslam_MagicSlamNative_nativeCreate(
    JNIEnv* env, jobject thiz,
    jfloat fx, jfloat fy, jfloat cx, jfloat cy,
    jint width, jint height,
    jint powerMode) {

    std::lock_guard<std::mutex> lock(g_mutex);

    if (g_slam) {
        LOGW("SLAM already created, destroying old instance");
        g_slam.reset();
    }

    try {
        Config config;

        // Camera intrinsics
        config.camera.fx = fx;
        config.camera.fy = fy;
        config.camera.cx = cx;
        config.camera.cy = cy;
        config.camera.width = width;
        config.camera.height = height;

        // Default distortion (caller should set real values)
        config.camera.k1 = config.camera.k2 = 0;
        config.camera.p1 = config.camera.p2 = 0;
        config.camera.k3 = 0;

        // Features optimized for AR1
        config.features.numFeatures = 500;
        config.features.numLevels = 4;
        config.features.scaleFactor = 1.2f;

        // Power mode
        switch (powerMode) {
            case 0:
                config.powerMode = Platform::PowerManager::PerformanceMode::LOW_POWER;
                break;
            case 1:
                config.powerMode = Platform::PowerManager::PerformanceMode::BALANCED;
                break;
            case 2:
                config.powerMode = Platform::PowerManager::PerformanceMode::HIGH_PERFORMANCE;
                break;
            default:
                config.powerMode = Platform::PowerManager::PerformanceMode::BALANCED;
        }

        g_slam = std::make_unique<SlamSystem>(config);

        // Set tracking callback
        g_slam->setTrackingCallback([](TrackingState state, const Pose& pose, Timestamp ts) {
            JNIEnv* env = getEnv();
            if (env && g_callbackObj && g_onTrackingStateChanged) {
                env->CallVoidMethod(g_callbackObj, g_onTrackingStateChanged,
                    static_cast<jint>(state),
                    toFloat(pose.position.x),
                    toFloat(pose.position.y),
                    toFloat(pose.position.z),
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z);
            }
        });

        LOGI("SLAM created: %dx%d, fx=%.1f, fy=%.1f", width, height, fx, fy);
        return JNI_TRUE;

    } catch (const std::exception& e) {
        LOGE("Failed to create SLAM: %s", e.what());
        return JNI_FALSE;
    }
}

/**
 * Set distortion coefficients
 */
JNIEXPORT void JNICALL
Java_com_magicslam_MagicSlamNative_nativeSetDistortion(
    JNIEnv* env, jobject thiz,
    jfloat k1, jfloat k2, jfloat p1, jfloat p2, jfloat k3) {
    // Note: In a real implementation, we'd need to reconfigure the system
    LOGI("Distortion set: k1=%.4f, k2=%.4f, p1=%.4f, p2=%.4f, k3=%.4f",
         k1, k2, p1, p2, k3);
}

/**
 * Process IMU measurement
 */
JNIEXPORT void JNICALL
Java_com_magicslam_MagicSlamNative_nativeProcessIMU(
    JNIEnv* env, jobject thiz,
    jlong timestamp,
    jfloat ax, jfloat ay, jfloat az,
    jfloat gx, jfloat gy, jfloat gz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return;

    ImuMeasurement imu;
    imu.timestamp = static_cast<Timestamp>(timestamp);
    imu.acceleration = Vec3{toFixed(ax), toFixed(ay), toFixed(az)};
    imu.angularVelocity = Vec3{toFixed(gx), toFixed(gy), toFixed(gz)};

    g_slam->processIMU(imu);
}

/**
 * Process camera frame (grayscale)
 */
JNIEXPORT void JNICALL
Java_com_magicslam_MagicSlamNative_nativeProcessFrame(
    JNIEnv* env, jobject thiz,
    jbyteArray imageData,
    jint width, jint height,
    jlong timestamp) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return;

    jbyte* data = env->GetByteArrayElements(imageData, nullptr);
    if (!data) {
        LOGE("Failed to get image data");
        return;
    }

    g_slam->processFrame(
        reinterpret_cast<const uint8_t*>(data),
        width, height,
        static_cast<Timestamp>(timestamp));

    env->ReleaseByteArrayElements(imageData, data, JNI_ABORT);
}

/**
 * Get current pose
 */
JNIEXPORT jfloatArray JNICALL
Java_com_magicslam_MagicSlamNative_nativeGetCurrentPose(
    JNIEnv* env, jobject thiz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return nullptr;

    Pose pose = g_slam->getCurrentPose();

    jfloatArray result = env->NewFloatArray(7);
    float poseData[7] = {
        toFloat(pose.position.x),
        toFloat(pose.position.y),
        toFloat(pose.position.z),
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    };
    env->SetFloatArrayRegion(result, 0, 7, poseData);
    return result;
}

/**
 * Get predicted pose for rendering
 */
JNIEXPORT jfloatArray JNICALL
Java_com_magicslam_MagicSlamNative_nativeGetPredictedPose(
    JNIEnv* env, jobject thiz,
    jlong renderTime) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return nullptr;

    Pose pose = g_slam->getPredictedPose(static_cast<Timestamp>(renderTime));

    jfloatArray result = env->NewFloatArray(7);
    float poseData[7] = {
        toFloat(pose.position.x),
        toFloat(pose.position.y),
        toFloat(pose.position.z),
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    };
    env->SetFloatArrayRegion(result, 0, 7, poseData);
    return result;
}

/**
 * Get tracking state
 */
JNIEXPORT jint JNICALL
Java_com_magicslam_MagicSlamNative_nativeGetTrackingState(
    JNIEnv* env, jobject thiz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return -1;

    return static_cast<jint>(g_slam->getTrackingState());
}

/**
 * Get tracking confidence (0-1)
 */
JNIEXPORT jfloat JNICALL
Java_com_magicslam_MagicSlamNative_nativeGetTrackingConfidence(
    JNIEnv* env, jobject thiz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return 0.0f;

    return g_slam->getTrackingConfidence();
}

/**
 * Create anchor at position
 */
JNIEXPORT jlong JNICALL
Java_com_magicslam_MagicSlamNative_nativeCreateAnchor(
    JNIEnv* env, jobject thiz,
    jfloat x, jfloat y, jfloat z) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return -1;

    Vec3 position{toFixed(x), toFixed(y), toFixed(z)};
    return static_cast<jlong>(g_slam->createAnchor(position));
}

/**
 * Get anchor pose
 */
JNIEXPORT jfloatArray JNICALL
Java_com_magicslam_MagicSlamNative_nativeGetAnchorPose(
    JNIEnv* env, jobject thiz,
    jlong anchorId) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return nullptr;

    const Anchor* anchor = g_slam->getAnchor(static_cast<int64_t>(anchorId));
    if (!anchor) return nullptr;

    jfloatArray result = env->NewFloatArray(8);
    float data[8] = {
        toFloat(anchor->pose.position.x),
        toFloat(anchor->pose.position.y),
        toFloat(anchor->pose.position.z),
        anchor->pose.orientation.w,
        anchor->pose.orientation.x,
        anchor->pose.orientation.y,
        anchor->pose.orientation.z,
        anchor->confidence
    };
    env->SetFloatArrayRegion(result, 0, 8, data);
    return result;
}

/**
 * Remove anchor
 */
JNIEXPORT void JNICALL
Java_com_magicslam_MagicSlamNative_nativeRemoveAnchor(
    JNIEnv* env, jobject thiz,
    jlong anchorId) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return;

    g_slam->removeAnchor(static_cast<int64_t>(anchorId));
}

/**
 * Get all detected planes
 */
JNIEXPORT jobjectArray JNICALL
Java_com_magicslam_MagicSlamNative_nativeGetPlanes(
    JNIEnv* env, jobject thiz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return nullptr;

    auto planes = g_slam->getPlanes();

    // Find PlaneData class
    jclass planeClass = env->FindClass("com/magicslam/PlaneData");
    if (!planeClass) return nullptr;

    jmethodID constructor = env->GetMethodID(planeClass, "<init>",
        "(JFFFFFFFZFF)V");
    if (!constructor) return nullptr;

    jobjectArray result = env->NewObjectArray(
        static_cast<jsize>(planes.size()), planeClass, nullptr);

    for (size_t i = 0; i < planes.size(); ++i) {
        const auto& p = planes[i];
        jobject plane = env->NewObject(planeClass, constructor,
            static_cast<jlong>(p.id),
            toFloat(p.center.x), toFloat(p.center.y), toFloat(p.center.z),
            toFloat(p.normal.x), toFloat(p.normal.y), toFloat(p.normal.z),
            p.isVertical ? JNI_TRUE : JNI_FALSE,
            p.width, p.height);
        env->SetObjectArrayElement(result, static_cast<jsize>(i), plane);
        env->DeleteLocalRef(plane);
    }

    return result;
}

/**
 * Get map statistics
 */
JNIEXPORT jintArray JNICALL
Java_com_magicslam_MagicSlamNative_nativeGetMapStats(
    JNIEnv* env, jobject thiz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_slam) return nullptr;

    jintArray result = env->NewIntArray(2);
    jint stats[2] = {
        static_cast<jint>(g_slam->getMapPointCount()),
        static_cast<jint>(g_slam->getKeyframeCount())
    };
    env->SetIntArrayRegion(result, 0, 2, stats);
    return result;
}

/**
 * Set callback object for events
 */
JNIEXPORT void JNICALL
Java_com_magicslam_MagicSlamNative_nativeSetCallback(
    JNIEnv* env, jobject thiz,
    jobject callback) {

    if (g_callbackObj) {
        env->DeleteGlobalRef(g_callbackObj);
        g_callbackObj = nullptr;
    }

    if (callback) {
        g_callbackObj = env->NewGlobalRef(callback);

        jclass callbackClass = env->GetObjectClass(callback);
        g_onTrackingStateChanged = env->GetMethodID(callbackClass,
            "onTrackingStateChanged", "(IFFFFFFF)V");
        g_onPlaneDetected = env->GetMethodID(callbackClass,
            "onPlaneDetected", "(JFFFFFFZFF)V");
    }
}

/**
 * Reset SLAM system
 */
JNIEXPORT void JNICALL
Java_com_magicslam_MagicSlamNative_nativeReset(
    JNIEnv* env, jobject thiz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_slam) {
        g_slam->reset();
        LOGI("SLAM reset");
    }
}

/**
 * Destroy SLAM system
 */
JNIEXPORT void JNICALL
Java_com_magicslam_MagicSlamNative_nativeDestroy(
    JNIEnv* env, jobject thiz) {

    std::lock_guard<std::mutex> lock(g_mutex);
    g_slam.reset();

    if (g_callbackObj) {
        env->DeleteGlobalRef(g_callbackObj);
        g_callbackObj = nullptr;
    }

    LOGI("SLAM destroyed");
}

}  // extern "C"
