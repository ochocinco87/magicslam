package com.magicslam.demo

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Color
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
import android.view.Surface
import android.view.View
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.magicslam.MagicSlam
import com.magicslam.Pose
import com.magicslam.AnchorData
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.util.concurrent.ConcurrentHashMap
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Stability Test Activity
 *
 * Demonstrates drift-free anchoring by:
 * 1. Placing a grid of virtual objects in physical space
 * 2. Tracking their positions over time
 * 3. Measuring and displaying any drift
 * 4. Showing that objects "stay in place" as user moves
 */
class StabilityTestActivity : AppCompatActivity(), GLSurfaceView.Renderer {

    companion object {
        private const val TAG = "StabilityTest"
        private const val CAMERA_PERMISSION_CODE = 100
        private const val CAMERA_WIDTH = 640
        private const val CAMERA_HEIGHT = 480
        private const val CAMERA_FX = 500f
        private const val CAMERA_FY = 500f

        // Test grid configuration
        private const val GRID_SIZE = 3        // 3x3 grid
        private const val GRID_SPACING = 0.3f  // 30cm between objects
        private const val OBJECT_DISTANCE = 1.5f // 1.5m in front
    }

    // SLAM
    private lateinit var slam: MagicSlam
    private var isTracking = false

    // UI
    private lateinit var glSurfaceView: GLSurfaceView
    private lateinit var statusText: TextView
    private lateinit var driftText: TextView
    private lateinit var instructionText: TextView
    private var renderer: ARRenderer? = null

    // Test state
    private enum class TestState {
        WAITING_FOR_TRACKING,
        PLACE_GRID,
        TESTING,
        COMPLETE
    }
    private var testState = TestState.WAITING_FOR_TRACKING

    // Anchored objects with initial positions for drift measurement
    data class TestAnchor(
        val anchorId: Long,
        val initialWorldPos: FloatArray,  // Position when created
        val gridX: Int,
        val gridY: Int,
        var maxDrift: Float = 0f,
        var currentDrift: Float = 0f,
        var measurements: Int = 0
    )

    private val testAnchors = ConcurrentHashMap<Long, TestAnchor>()
    private var testStartTime = 0L
    private var totalDriftSum = 0f
    private var driftMeasurements = 0

    // Camera
    private var cameraDevice: CameraDevice? = null
    private var cameraCaptureSession: CameraCaptureSession? = null
    private var cameraHandler: Handler? = null
    private var cameraThread: HandlerThread? = null

    // Current pose
    private var currentPose: Pose? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setupUI()
        initializeSLAM()

        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_CODE)
        } else {
            startCamera()
        }
    }

    private fun setupUI() {
        val mainLayout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
        }

        // GL Surface
        glSurfaceView = GLSurfaceView(this).apply {
            setEGLContextClientVersion(2)
            setRenderer(this@StabilityTestActivity)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }

        // Control panel
        val controlPanel = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setBackgroundColor(0xCC000000.toInt())
            setPadding(16, 8, 16, 8)
        }

        // Status text
        statusText = TextView(this).apply {
            setTextColor(Color.WHITE)
            textSize = 12f
            text = "Status: Initializing..."
        }

        // Drift measurement text
        driftText = TextView(this).apply {
            setTextColor(Color.CYAN)
            textSize = 14f
            text = "Drift: --"
        }

        // Place grid button
        val placeButton = Button(this).apply {
            text = "Place Test Grid"
            setOnClickListener {
                if (testState == TestState.PLACE_GRID) {
                    placeTestGrid()
                }
            }
        }

        // Reset button
        val resetButton = Button(this).apply {
            text = "Reset Test"
            setOnClickListener { resetTest() }
        }

        controlPanel.addView(statusText, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        controlPanel.addView(driftText, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        controlPanel.addView(placeButton)
        controlPanel.addView(resetButton)

        // Instruction overlay
        instructionText = TextView(this).apply {
            setTextColor(Color.YELLOW)
            setBackgroundColor(0x80000000.toInt())
            textSize = 18f
            setPadding(32, 32, 32, 32)
            text = "Move around to initialize tracking..."
            visibility = View.VISIBLE
        }

        // Layout
        val frameLayout = android.widget.FrameLayout(this)
        frameLayout.addView(glSurfaceView)
        frameLayout.addView(instructionText, android.widget.FrameLayout.LayoutParams(
            android.widget.FrameLayout.LayoutParams.WRAP_CONTENT,
            android.widget.FrameLayout.LayoutParams.WRAP_CONTENT
        ).apply {
            gravity = android.view.Gravity.CENTER
        })

        mainLayout.addView(frameLayout, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f))
        mainLayout.addView(controlPanel)

        setContentView(mainLayout)
    }

    private fun initializeSLAM() {
        slam = MagicSlam(this)

        val success = slam.create(
            fx = CAMERA_FX,
            fy = CAMERA_FY,
            cx = CAMERA_WIDTH / 2f,
            cy = CAMERA_HEIGHT / 2f,
            width = CAMERA_WIDTH,
            height = CAMERA_HEIGHT,
            powerMode = MagicSlam.PowerMode.HIGH_PERFORMANCE
        )

        if (!success) {
            Log.e(TAG, "Failed to initialize SLAM")
            finish()
            return
        }

        slam.setCallback(object : MagicSlam.Callback {
            override fun onTrackingStateChanged(
                state: MagicSlam.TrackingState,
                posX: Float, posY: Float, posZ: Float,
                oriW: Float, oriX: Float, oriY: Float, oriZ: Float
            ) {
                isTracking = state == MagicSlam.TrackingState.TRACKING
                currentPose = Pose(posX, posY, posZ, oriW, oriX, oriY, oriZ)

                runOnUiThread {
                    when (state) {
                        MagicSlam.TrackingState.TRACKING -> {
                            if (testState == TestState.WAITING_FOR_TRACKING) {
                                testState = TestState.PLACE_GRID
                                instructionText.text = "Tracking! Press 'Place Test Grid' to begin test."
                            }
                        }
                        MagicSlam.TrackingState.LOST -> {
                            instructionText.text = "Tracking lost! Move slowly..."
                            instructionText.visibility = View.VISIBLE
                        }
                        else -> {}
                    }
                    updateStatus()
                }

                // Measure drift if testing
                if (testState == TestState.TESTING) {
                    measureDrift()
                }
            }

            override fun onPlaneDetected(plane: com.magicslam.PlaneData) {}
        })

        slam.startIMU()
    }

    private fun placeTestGrid() {
        val pose = currentPose ?: return

        testAnchors.clear()
        testState = TestState.TESTING
        testStartTime = System.currentTimeMillis()

        // Place a grid of objects in front of the camera
        val halfGrid = GRID_SIZE / 2

        for (gridY in 0 until GRID_SIZE) {
            for (gridX in 0 until GRID_SIZE) {
                // Calculate local position (relative to camera)
                val localX = (gridX - halfGrid) * GRID_SPACING
                val localY = (gridY - halfGrid) * GRID_SPACING
                val localZ = -OBJECT_DISTANCE

                // Transform to world coordinates
                val worldPos = transformToWorld(localX, localY, localZ, pose)

                // Create anchor
                val anchorId = slam.createAnchor(worldPos[0], worldPos[1], worldPos[2])

                if (anchorId >= 0) {
                    testAnchors[anchorId] = TestAnchor(
                        anchorId = anchorId,
                        initialWorldPos = worldPos.clone(),
                        gridX = gridX,
                        gridY = gridY
                    )
                }
            }
        }

        runOnUiThread {
            instructionText.text = "Grid placed! Walk around the objects.\nThey should stay in place."
            instructionText.visibility = View.VISIBLE

            // Hide instruction after 3 seconds
            instructionText.postDelayed({
                instructionText.visibility = View.GONE
            }, 3000)
        }

        Log.i(TAG, "Placed ${testAnchors.size} test anchors")
    }

    private fun measureDrift() {
        for ((anchorId, testAnchor) in testAnchors) {
            val anchorData = slam.getAnchorPose(anchorId) ?: continue

            // Calculate drift from initial position
            val dx = anchorData.pose.x - testAnchor.initialWorldPos[0]
            val dy = anchorData.pose.y - testAnchor.initialWorldPos[1]
            val dz = anchorData.pose.z - testAnchor.initialWorldPos[2]

            val drift = sqrt(dx * dx + dy * dy + dz * dz)

            testAnchor.currentDrift = drift
            testAnchor.maxDrift = maxOf(testAnchor.maxDrift, drift)
            testAnchor.measurements++

            totalDriftSum += drift
            driftMeasurements++
        }

        runOnUiThread { updateDriftDisplay() }
    }

    private fun updateDriftDisplay() {
        if (testAnchors.isEmpty()) {
            driftText.text = "Drift: --"
            return
        }

        val avgDrift = if (driftMeasurements > 0) totalDriftSum / driftMeasurements else 0f
        val maxDrift = testAnchors.values.maxOfOrNull { it.maxDrift } ?: 0f
        val currentMaxDrift = testAnchors.values.maxOfOrNull { it.currentDrift } ?: 0f

        val testDuration = (System.currentTimeMillis() - testStartTime) / 1000

        val driftMm = currentMaxDrift * 1000  // Convert to mm
        val avgDriftMm = avgDrift * 1000
        val maxDriftMm = maxDrift * 1000

        // Color code based on drift quality
        val color = when {
            maxDriftMm < 5 -> Color.GREEN      // Excellent: < 5mm
            maxDriftMm < 20 -> Color.YELLOW    // Good: < 20mm
            maxDriftMm < 50 -> 0xFFFFA500.toInt() // Orange: < 50mm
            else -> Color.RED                   // Poor: > 50mm
        }

        driftText.setTextColor(color)
        driftText.text = String.format(
            "Drift: %.1fmm (avg: %.1fmm, max: %.1fmm) | Time: %ds",
            driftMm, avgDriftMm, maxDriftMm, testDuration
        )
    }

    private fun updateStatus() {
        val state = slam.getTrackingState()
        val confidence = (slam.getTrackingConfidence() * 100).toInt()
        val stats = slam.getMapStats()

        statusText.text = String.format(
            "%s | Conf: %d%% | Map: %d pts | Anchors: %d",
            state.name, confidence, stats.first, testAnchors.size
        )
    }

    private fun resetTest() {
        // Remove all anchors
        for (anchorId in testAnchors.keys) {
            slam.removeAnchor(anchorId)
        }
        testAnchors.clear()

        totalDriftSum = 0f
        driftMeasurements = 0
        testState = if (isTracking) TestState.PLACE_GRID else TestState.WAITING_FOR_TRACKING

        runOnUiThread {
            driftText.text = "Drift: --"
            instructionText.text = if (isTracking)
                "Press 'Place Test Grid' to begin test."
            else
                "Move around to initialize tracking..."
            instructionText.visibility = View.VISIBLE
        }
    }

    private fun transformToWorld(x: Float, y: Float, z: Float, pose: Pose): FloatArray {
        val qw = pose.qw
        val qx = pose.qx
        val qy = pose.qy
        val qz = pose.qz

        val uvX = qy * z - qz * y
        val uvY = qz * x - qx * z
        val uvZ = qx * y - qy * x

        val uuvX = qy * uvZ - qz * uvY
        val uuvY = qz * uvX - qx * uvZ
        val uuvZ = qx * uvY - qy * uvX

        val rotX = x + 2f * (qw * uvX + uuvX)
        val rotY = y + 2f * (qw * uvY + uuvY)
        val rotZ = z + 2f * (qw * uvZ + uuvZ)

        return floatArrayOf(rotX + pose.x, rotY + pose.y, rotZ + pose.z)
    }

    // Camera handling (same as ARDemoActivity)
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
                    override fun onDisconnected(camera: CameraDevice) { camera.close() }
                    override fun onError(camera: CameraDevice, error: Int) { camera.close() }
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
                CAMERA_WIDTH, CAMERA_HEIGHT, ImageFormat.YUV_420_888, 2)

            imageReader.setOnImageAvailableListener({ reader ->
                val image = reader.acquireLatestImage() ?: return@setOnImageAvailableListener
                val yBuffer = image.planes[0].buffer
                val yData = ByteArray(yBuffer.remaining())
                yBuffer.get(yData)
                slam.processFrame(yData, image.width, image.height, image.timestamp)
                image.close()
            }, cameraHandler)

            camera.createCaptureSession(listOf(surface, imageReader.surface),
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        cameraCaptureSession = session
                        val captureRequest = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                            addTarget(imageReader.surface)
                        }.build()
                        session.setRepeatingRequest(captureRequest, null, cameraHandler)
                    }
                    override fun onConfigureFailed(session: CameraCaptureSession) {}
                }, cameraHandler)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to create capture session", e)
        }
    }

    // GLSurfaceView.Renderer
    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.05f, 0.05f, 0.1f, 1.0f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        renderer = ARRenderer()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        renderer?.setViewport(width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        val renderTime = System.nanoTime() + 15_000_000L
        val pose = slam.getPredictedPose(renderTime) ?: return

        renderer?.let { r ->
            r.setCameraPose(pose)

            // Draw world origin axes
            r.drawAxes(0f, 0f, 0f, 0.5f)

            // Draw test grid objects with color-coded drift
            for ((anchorId, testAnchor) in testAnchors) {
                val anchorData = slam.getAnchorPose(anchorId) ?: continue

                // Color based on drift: green = stable, red = drifting
                val driftMm = testAnchor.currentDrift * 1000
                val color = when {
                    driftMm < 5 -> floatArrayOf(0f, 1f, 0f, 1f)       // Green
                    driftMm < 20 -> floatArrayOf(1f, 1f, 0f, 1f)      // Yellow
                    driftMm < 50 -> floatArrayOf(1f, 0.5f, 0f, 1f)    // Orange
                    else -> floatArrayOf(1f, 0f, 0f, 1f)               // Red
                }

                val obj = VirtualObject(
                    anchorId = anchorId,
                    type = VirtualObjectType.CUBE,
                    color = color,
                    scale = 0.05f  // 5cm cubes
                )

                r.drawObject(obj, anchorData.pose, anchorData.confidence)
            }
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == CAMERA_PERMISSION_CODE && grantResults.isNotEmpty() &&
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
