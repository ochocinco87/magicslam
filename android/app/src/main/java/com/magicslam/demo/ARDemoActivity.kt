package com.magicslam.demo

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.graphics.SurfaceTexture
import android.hardware.camera2.*
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import android.view.MotionEvent
import android.view.Surface
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.magicslam.MagicSlam
import com.magicslam.Pose
import com.magicslam.PlaneData
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.util.concurrent.ConcurrentHashMap
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.cos
import kotlin.math.sin

/**
 * AR Demo Activity - Places virtual objects in physical space
 *
 * Demonstrates:
 * - Multiple virtual objects anchored to real-world positions
 * - Stable tracking as user moves around
 * - Plane detection for surface placement
 * - Drift-free anchoring with MagicSLAM
 */
class ARDemoActivity : AppCompatActivity(), GLSurfaceView.Renderer {

    companion object {
        private const val TAG = "ARDemo"
        private const val CAMERA_PERMISSION_CODE = 100

        // Camera settings (adjust for X3 Pro)
        private const val CAMERA_WIDTH = 640
        private const val CAMERA_HEIGHT = 480
        private const val CAMERA_FX = 500f
        private const val CAMERA_FY = 500f
    }

    // SLAM system
    private lateinit var slam: MagicSlam
    private var isTracking = false

    // Rendering
    private lateinit var glSurfaceView: GLSurfaceView
    private lateinit var statusText: TextView
    private var renderer: ARRenderer? = null

    // Virtual objects
    private val virtualObjects = ConcurrentHashMap<Long, VirtualObject>()
    private var nextObjectId = 0L

    // Camera
    private var cameraDevice: CameraDevice? = null
    private var cameraCaptureSession: CameraCaptureSession? = null
    private var cameraHandler: Handler? = null
    private var cameraThread: HandlerThread? = null

    // Tracking state
    private var currentPose: Pose? = null
    private var trackingConfidence = 0f
    private var mapPointCount = 0
    private var detectedPlanes = listOf<PlaneData>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Setup UI
        setupUI()

        // Initialize SLAM
        initializeSLAM()

        // Request permissions
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_CODE)
        } else {
            startCamera()
        }
    }

    private fun setupUI() {
        // Create layout programmatically
        val layout = android.widget.FrameLayout(this)

        // GL Surface for AR rendering
        glSurfaceView = GLSurfaceView(this).apply {
            setEGLContextClientVersion(2)
            setRenderer(this@ARDemoActivity)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
        layout.addView(glSurfaceView)

        // Status overlay
        statusText = TextView(this).apply {
            setTextColor(0xFFFFFFFF.toInt())
            setBackgroundColor(0x80000000.toInt())
            setPadding(16, 16, 16, 16)
            textSize = 14f
            text = "Initializing SLAM..."
        }
        layout.addView(statusText, android.widget.FrameLayout.LayoutParams(
            android.widget.FrameLayout.LayoutParams.WRAP_CONTENT,
            android.widget.FrameLayout.LayoutParams.WRAP_CONTENT
        ))

        setContentView(layout)

        // Tap to place objects
        glSurfaceView.setOnTouchListener { _, event ->
            if (event.action == MotionEvent.ACTION_DOWN && isTracking) {
                placeObjectAtTap(event.x, event.y)
            }
            true
        }
    }

    private fun initializeSLAM() {
        slam = MagicSlam(this)

        // Create with X3 Pro camera parameters
        val success = slam.create(
            fx = CAMERA_FX,
            fy = CAMERA_FY,
            cx = CAMERA_WIDTH / 2f,
            cy = CAMERA_HEIGHT / 2f,
            width = CAMERA_WIDTH,
            height = CAMERA_HEIGHT,
            powerMode = MagicSlam.PowerMode.BALANCED
        )

        if (!success) {
            Toast.makeText(this, "Failed to initialize SLAM", Toast.LENGTH_LONG).show()
            finish()
            return
        }

        // Set callback for tracking updates
        slam.setCallback(object : MagicSlam.Callback {
            override fun onTrackingStateChanged(
                state: MagicSlam.TrackingState,
                posX: Float, posY: Float, posZ: Float,
                oriW: Float, oriX: Float, oriY: Float, oriZ: Float
            ) {
                isTracking = state == MagicSlam.TrackingState.TRACKING
                currentPose = Pose(posX, posY, posZ, oriW, oriX, oriY, oriZ)
                trackingConfidence = slam.getTrackingConfidence()

                runOnUiThread { updateStatus() }
            }

            override fun onPlaneDetected(plane: PlaneData) {
                detectedPlanes = slam.getPlanes()
                runOnUiThread { updateStatus() }
            }
        })

        // Start IMU
        slam.startIMU()

        Log.i(TAG, "SLAM initialized")
    }

    private fun placeObjectAtTap(screenX: Float, screenY: Float) {
        val pose = currentPose ?: return

        // Ray from camera through tap point
        val ndcX = (screenX / glSurfaceView.width) * 2f - 1f
        val ndcY = 1f - (screenY / glSurfaceView.height) * 2f

        // Place object 1.5m in front of camera
        val distance = 1.5f
        val localX = ndcX * distance * 0.5f
        val localY = ndcY * distance * 0.5f
        val localZ = -distance

        // Transform to world space using current pose
        val worldPos = transformToWorld(localX, localY, localZ, pose)

        // Create anchor
        val anchorId = slam.createAnchor(worldPos[0], worldPos[1], worldPos[2])

        if (anchorId >= 0) {
            // Create virtual object at anchor
            val obj = VirtualObject(
                anchorId = anchorId,
                type = VirtualObjectType.values()[nextObjectId.toInt() % VirtualObjectType.values().size],
                color = OBJECT_COLORS[(nextObjectId % OBJECT_COLORS.size).toInt()],
                scale = 0.1f + (nextObjectId % 3) * 0.05f
            )
            virtualObjects[anchorId] = obj
            nextObjectId++

            runOnUiThread {
                Toast.makeText(this, "Placed ${obj.type.name} #$anchorId", Toast.LENGTH_SHORT).show()
                updateStatus()
            }
        }
    }

    private fun transformToWorld(x: Float, y: Float, z: Float, pose: Pose): FloatArray {
        // Rotate by quaternion and add position
        val qw = pose.qw
        val qx = pose.qx
        val qy = pose.qy
        val qz = pose.qz

        // Quaternion rotation: v' = q * v * q^-1 (optimized)
        val uvX = qy * z - qz * y
        val uvY = qz * x - qx * z
        val uvZ = qx * y - qy * x

        val uuvX = qy * uvZ - qz * uvY
        val uuvY = qz * uvX - qx * uvZ
        val uuvZ = qx * uvY - qy * uvX

        val rotX = x + 2f * (qw * uvX + uuvX)
        val rotY = y + 2f * (qw * uvY + uuvY)
        val rotZ = z + 2f * (qw * uvZ + uuvZ)

        return floatArrayOf(
            rotX + pose.x,
            rotY + pose.y,
            rotZ + pose.z
        )
    }

    private fun updateStatus() {
        val state = slam.getTrackingState()
        val stats = slam.getMapStats()
        mapPointCount = stats.first

        val statusBuilder = StringBuilder()
        statusBuilder.append("State: ${state.name}\n")
        statusBuilder.append("Confidence: ${(trackingConfidence * 100).toInt()}%\n")
        statusBuilder.append("Map Points: $mapPointCount\n")
        statusBuilder.append("Keyframes: ${stats.second}\n")
        statusBuilder.append("Planes: ${detectedPlanes.size}\n")
        statusBuilder.append("Objects: ${virtualObjects.size}\n")

        currentPose?.let { p ->
            statusBuilder.append(String.format("Pos: (%.2f, %.2f, %.2f)\n", p.x, p.y, p.z))
        }

        statusBuilder.append("\n[Tap to place objects]")

        statusText.text = statusBuilder.toString()
    }

    // Camera handling
    private fun startCamera() {
        cameraThread = HandlerThread("CameraThread").apply { start() }
        cameraHandler = Handler(cameraThread!!.looper)

        val cameraManager = getSystemService(CAMERA_SERVICE) as CameraManager
        try {
            val cameraId = cameraManager.cameraIdList[0]

            if (checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                cameraManager.openCamera(cameraId, object : CameraDevice.StateCallback() {
                    override fun onOpened(camera: CameraDevice) {
                        cameraDevice = camera
                        createCaptureSession()
                    }

                    override fun onDisconnected(camera: CameraDevice) {
                        camera.close()
                    }

                    override fun onError(camera: CameraDevice, error: Int) {
                        Log.e(TAG, "Camera error: $error")
                        camera.close()
                    }
                }, cameraHandler)
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to open camera", e)
        }
    }

    private fun createCaptureSession() {
        val camera = cameraDevice ?: return

        try {
            val surfaceTexture = SurfaceTexture(0)
            surfaceTexture.setDefaultBufferSize(CAMERA_WIDTH, CAMERA_HEIGHT)
            val surface = Surface(surfaceTexture)

            val imageReader = android.media.ImageReader.newInstance(
                CAMERA_WIDTH, CAMERA_HEIGHT, ImageFormat.YUV_420_888, 2
            )

            imageReader.setOnImageAvailableListener({ reader ->
                val image = reader.acquireLatestImage() ?: return@setOnImageAvailableListener

                // Extract Y plane (grayscale)
                val yBuffer = image.planes[0].buffer
                val yData = ByteArray(yBuffer.remaining())
                yBuffer.get(yData)

                // Process frame
                slam.processFrame(yData, image.width, image.height, image.timestamp)

                image.close()
            }, cameraHandler)

            camera.createCaptureSession(
                listOf(surface, imageReader.surface),
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        cameraCaptureSession = session

                        val captureRequest = camera.createCaptureRequest(
                            CameraDevice.TEMPLATE_PREVIEW
                        ).apply {
                            addTarget(imageReader.surface)
                            set(CaptureRequest.CONTROL_AF_MODE,
                                CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
                        }.build()

                        session.setRepeatingRequest(captureRequest, null, cameraHandler)
                    }

                    override fun onConfigureFailed(session: CameraCaptureSession) {
                        Log.e(TAG, "Camera session configuration failed")
                    }
                },
                cameraHandler
            )
        } catch (e: Exception) {
            Log.e(TAG, "Failed to create capture session", e)
        }
    }

    // GLSurfaceView.Renderer implementation
    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        renderer = ARRenderer()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        renderer?.setViewport(width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        // Get predicted pose for rendering (15ms lookahead)
        val renderTime = System.nanoTime() + 15_000_000L
        val pose = slam.getPredictedPose(renderTime) ?: return

        renderer?.let { r ->
            // Set camera from pose
            r.setCameraPose(pose)

            // Draw detected planes
            for (plane in detectedPlanes) {
                r.drawPlane(plane)
            }

            // Draw anchored virtual objects
            for ((anchorId, obj) in virtualObjects) {
                val anchorData = slam.getAnchorPose(anchorId)
                if (anchorData != null && anchorData.confidence > 0.3f) {
                    r.drawObject(obj, anchorData.pose, anchorData.confidence)
                }
            }

            // Draw coordinate axes at origin
            r.drawAxes(0f, 0f, 0f, 0.3f)
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == CAMERA_PERMISSION_CODE &&
            grantResults.isNotEmpty() &&
            grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startCamera()
        }
    }

    override fun onResume() {
        super.onResume()
        glSurfaceView.onResume()
    }

    override fun onPause() {
        super.onPause()
        glSurfaceView.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraCaptureSession?.close()
        cameraDevice?.close()
        cameraThread?.quitSafely()
        slam.destroy()
    }
}

// Virtual object types
enum class VirtualObjectType {
    CUBE,
    SPHERE,
    PYRAMID,
    CYLINDER,
    TORUS
}

// Virtual object data
data class VirtualObject(
    val anchorId: Long,
    val type: VirtualObjectType,
    val color: FloatArray,
    val scale: Float
)

// Object colors
val OBJECT_COLORS = arrayOf(
    floatArrayOf(1f, 0.2f, 0.2f, 1f),   // Red
    floatArrayOf(0.2f, 1f, 0.2f, 1f),   // Green
    floatArrayOf(0.2f, 0.2f, 1f, 1f),   // Blue
    floatArrayOf(1f, 1f, 0.2f, 1f),     // Yellow
    floatArrayOf(1f, 0.2f, 1f, 1f),     // Magenta
    floatArrayOf(0.2f, 1f, 1f, 1f),     // Cyan
    floatArrayOf(1f, 0.5f, 0.2f, 1f),   // Orange
    floatArrayOf(0.5f, 0.2f, 1f, 1f)    // Purple
)
