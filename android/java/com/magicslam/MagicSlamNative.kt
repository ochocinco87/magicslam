package com.magicslam

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log

/**
 * Kotlin wrapper for MagicSLAM native library
 *
 * Usage:
 * ```
 * val slam = MagicSlam(context)
 * slam.create(500f, 500f, 320f, 240f, 640, 480)
 * slam.startIMU()
 *
 * // In camera callback:
 * slam.processFrame(imageBytes, width, height, timestamp)
 *
 * // For rendering:
 * val pose = slam.getPredictedPose(System.nanoTime() + 15_000_000)
 * ```
 */
class MagicSlam(private val context: Context) : SensorEventListener {

    companion object {
        private const val TAG = "MagicSlam"

        init {
            System.loadLibrary("magicslam_jni")
        }
    }

    // Power modes
    enum class PowerMode(val value: Int) {
        LOW_POWER(0),
        BALANCED(1),
        HIGH_PERFORMANCE(2)
    }

    // Tracking states
    enum class TrackingState(val value: Int) {
        NOT_INITIALIZED(0),
        INITIALIZING(1),
        TRACKING(2),
        LOST(3),
        RELOCALIZATION(4);

        companion object {
            fun fromInt(value: Int) = entries.firstOrNull { it.value == value }
        }
    }

    // Callback interface
    interface Callback {
        fun onTrackingStateChanged(
            state: TrackingState,
            posX: Float, posY: Float, posZ: Float,
            oriW: Float, oriX: Float, oriY: Float, oriZ: Float
        )
        fun onPlaneDetected(plane: PlaneData)
    }

    private var sensorManager: SensorManager? = null
    private var lastAccel = floatArrayOf(0f, 0f, 0f)
    private var callback: Callback? = null
    private var isRunning = false

    /**
     * Create SLAM system with camera intrinsics
     */
    fun create(
        fx: Float, fy: Float,
        cx: Float, cy: Float,
        width: Int, height: Int,
        powerMode: PowerMode = PowerMode.BALANCED
    ): Boolean {
        Log.i(TAG, "Creating SLAM: ${width}x${height}, fx=$fx")
        return nativeCreate(fx, fy, cx, cy, width, height, powerMode.value)
    }

    /**
     * Set lens distortion coefficients
     */
    fun setDistortion(k1: Float, k2: Float, p1: Float, p2: Float, k3: Float = 0f) {
        nativeSetDistortion(k1, k2, p1, p2, k3)
    }

    /**
     * Start IMU sensor processing
     */
    fun startIMU() {
        if (isRunning) return

        sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

        val accel = sensorManager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyro = sensorManager?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        accel?.let {
            sensorManager?.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST)
        }
        gyro?.let {
            sensorManager?.registerListener(this, it, SensorManager.SENSOR_DELAY_FASTEST)
        }

        isRunning = true
        Log.i(TAG, "IMU started")
    }

    /**
     * Stop IMU sensor processing
     */
    fun stopIMU() {
        sensorManager?.unregisterListener(this)
        isRunning = false
        Log.i(TAG, "IMU stopped")
    }

    /**
     * Process camera frame (grayscale Y plane)
     */
    fun processFrame(imageData: ByteArray, width: Int, height: Int, timestamp: Long) {
        nativeProcessFrame(imageData, width, height, timestamp)
    }

    /**
     * Get current tracked pose
     * Returns [x, y, z, qw, qx, qy, qz] or null if not tracking
     */
    fun getCurrentPose(): Pose? {
        val data = nativeGetCurrentPose() ?: return null
        return Pose(
            x = data[0], y = data[1], z = data[2],
            qw = data[3], qx = data[4], qy = data[5], qz = data[6]
        )
    }

    /**
     * Get predicted pose for rendering (compensates for latency)
     * @param renderTime Target render timestamp in nanoseconds
     */
    fun getPredictedPose(renderTime: Long): Pose? {
        val data = nativeGetPredictedPose(renderTime) ?: return null
        return Pose(
            x = data[0], y = data[1], z = data[2],
            qw = data[3], qx = data[4], qy = data[5], qz = data[6]
        )
    }

    /**
     * Get current tracking state
     */
    fun getTrackingState(): TrackingState {
        return TrackingState.fromInt(nativeGetTrackingState()) ?: TrackingState.NOT_INITIALIZED
    }

    /**
     * Get tracking confidence (0-1)
     */
    fun getTrackingConfidence(): Float {
        return nativeGetTrackingConfidence()
    }

    /**
     * Create anchor at world position
     * @return Anchor ID or -1 on failure
     */
    fun createAnchor(x: Float, y: Float, z: Float): Long {
        return nativeCreateAnchor(x, y, z)
    }

    /**
     * Get anchor pose and confidence
     */
    fun getAnchorPose(anchorId: Long): AnchorData? {
        val data = nativeGetAnchorPose(anchorId) ?: return null
        return AnchorData(
            id = anchorId,
            pose = Pose(
                x = data[0], y = data[1], z = data[2],
                qw = data[3], qx = data[4], qy = data[5], qz = data[6]
            ),
            confidence = data[7]
        )
    }

    /**
     * Remove anchor
     */
    fun removeAnchor(anchorId: Long) {
        nativeRemoveAnchor(anchorId)
    }

    /**
     * Get all detected planes
     */
    fun getPlanes(): List<PlaneData> {
        return nativeGetPlanes()?.toList() ?: emptyList()
    }

    /**
     * Get map statistics
     * @return Pair of (mapPointCount, keyframeCount)
     */
    fun getMapStats(): Pair<Int, Int> {
        val stats = nativeGetMapStats()
        return if (stats != null && stats.size >= 2) {
            Pair(stats[0], stats[1])
        } else {
            Pair(0, 0)
        }
    }

    /**
     * Set callback for SLAM events
     */
    fun setCallback(callback: Callback) {
        this.callback = callback
        nativeSetCallback(object : NativeCallback {
            override fun onTrackingStateChanged(
                state: Int,
                posX: Float, posY: Float, posZ: Float,
                oriW: Float, oriX: Float, oriY: Float, oriZ: Float
            ) {
                callback.onTrackingStateChanged(
                    TrackingState.fromInt(state) ?: TrackingState.NOT_INITIALIZED,
                    posX, posY, posZ, oriW, oriX, oriY, oriZ
                )
            }

            override fun onPlaneDetected(
                id: Long,
                cx: Float, cy: Float, cz: Float,
                nx: Float, ny: Float, nz: Float,
                isVertical: Boolean,
                width: Float, height: Float
            ) {
                callback.onPlaneDetected(PlaneData(
                    id, cx, cy, cz, nx, ny, nz, isVertical, width, height
                ))
            }
        })
    }

    /**
     * Reset SLAM (clears map and tracking)
     */
    fun reset() {
        nativeReset()
    }

    /**
     * Destroy SLAM system and release resources
     */
    fun destroy() {
        stopIMU()
        nativeDestroy()
    }

    // SensorEventListener implementation
    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                lastAccel = event.values.clone()
            }
            Sensor.TYPE_GYROSCOPE -> {
                nativeProcessIMU(
                    event.timestamp,
                    lastAccel[0], lastAccel[1], lastAccel[2],
                    event.values[0], event.values[1], event.values[2]
                )
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    // Native methods
    private external fun nativeCreate(
        fx: Float, fy: Float, cx: Float, cy: Float,
        width: Int, height: Int, powerMode: Int
    ): Boolean

    private external fun nativeSetDistortion(
        k1: Float, k2: Float, p1: Float, p2: Float, k3: Float
    )

    private external fun nativeProcessIMU(
        timestamp: Long,
        ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float
    )

    private external fun nativeProcessFrame(
        imageData: ByteArray, width: Int, height: Int, timestamp: Long
    )

    private external fun nativeGetCurrentPose(): FloatArray?
    private external fun nativeGetPredictedPose(renderTime: Long): FloatArray?
    private external fun nativeGetTrackingState(): Int
    private external fun nativeGetTrackingConfidence(): Float
    private external fun nativeCreateAnchor(x: Float, y: Float, z: Float): Long
    private external fun nativeGetAnchorPose(anchorId: Long): FloatArray?
    private external fun nativeRemoveAnchor(anchorId: Long)
    private external fun nativeGetPlanes(): Array<PlaneData>?
    private external fun nativeGetMapStats(): IntArray?
    private external fun nativeSetCallback(callback: NativeCallback)
    private external fun nativeReset()
    private external fun nativeDestroy()

    // Internal callback interface for JNI
    private interface NativeCallback {
        fun onTrackingStateChanged(
            state: Int,
            posX: Float, posY: Float, posZ: Float,
            oriW: Float, oriX: Float, oriY: Float, oriZ: Float
        )
        fun onPlaneDetected(
            id: Long,
            cx: Float, cy: Float, cz: Float,
            nx: Float, ny: Float, nz: Float,
            isVertical: Boolean,
            width: Float, height: Float
        )
    }
}

/**
 * Pose data class
 */
data class Pose(
    val x: Float,
    val y: Float,
    val z: Float,
    val qw: Float,
    val qx: Float,
    val qy: Float,
    val qz: Float
) {
    fun toFloatArray() = floatArrayOf(x, y, z, qw, qx, qy, qz)

    fun toMatrix(): FloatArray {
        val m = FloatArray(16)
        // Convert quaternion to 4x4 matrix (column-major for OpenGL)
        val xx = qx * qx
        val yy = qy * qy
        val zz = qz * qz
        val xy = qx * qy
        val xz = qx * qz
        val yz = qy * qz
        val wx = qw * qx
        val wy = qw * qy
        val wz = qw * qz

        m[0] = 1 - 2 * (yy + zz)
        m[1] = 2 * (xy + wz)
        m[2] = 2 * (xz - wy)
        m[3] = 0f

        m[4] = 2 * (xy - wz)
        m[5] = 1 - 2 * (xx + zz)
        m[6] = 2 * (yz + wx)
        m[7] = 0f

        m[8] = 2 * (xz + wy)
        m[9] = 2 * (yz - wx)
        m[10] = 1 - 2 * (xx + yy)
        m[11] = 0f

        m[12] = x
        m[13] = y
        m[14] = z
        m[15] = 1f

        return m
    }
}

/**
 * Anchor data class
 */
data class AnchorData(
    val id: Long,
    val pose: Pose,
    val confidence: Float
)

/**
 * Plane data class
 */
data class PlaneData(
    val id: Long,
    val centerX: Float,
    val centerY: Float,
    val centerZ: Float,
    val normalX: Float,
    val normalY: Float,
    val normalZ: Float,
    val isVertical: Boolean,
    val width: Float,
    val height: Float
)
