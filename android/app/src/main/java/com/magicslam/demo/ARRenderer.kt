package com.magicslam.demo

import android.opengl.GLES20
import android.opengl.Matrix
import com.magicslam.PlaneData
import com.magicslam.Pose
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.ShortBuffer
import kotlin.math.cos
import kotlin.math.sin

/**
 * OpenGL ES 2.0 Renderer for AR content
 *
 * Renders:
 * - Virtual objects (cube, sphere, pyramid, etc.)
 * - Detected planes with grid overlay
 * - Coordinate axes for debugging
 * - Confidence-based transparency
 */
class ARRenderer {

    // Shader program
    private var objectProgram = 0
    private var lineProgram = 0

    // Matrices
    private val viewMatrix = FloatArray(16)
    private val projectionMatrix = FloatArray(16)
    private val mvpMatrix = FloatArray(16)
    private val modelMatrix = FloatArray(16)
    private val tempMatrix = FloatArray(16)

    // Geometry buffers
    private lateinit var cubeVertices: FloatBuffer
    private lateinit var cubeNormals: FloatBuffer
    private lateinit var cubeIndices: ShortBuffer

    private lateinit var sphereVertices: FloatBuffer
    private lateinit var sphereNormals: FloatBuffer
    private lateinit var sphereIndices: ShortBuffer

    private lateinit var pyramidVertices: FloatBuffer
    private lateinit var pyramidNormals: FloatBuffer

    private lateinit var planeVertices: FloatBuffer
    private lateinit var axesVertices: FloatBuffer
    private lateinit var axesColors: FloatBuffer

    // Counts
    private var sphereIndexCount = 0

    init {
        createShaders()
        createGeometry()
    }

    private fun createShaders() {
        // Object shader with lighting
        val objectVS = """
            uniform mat4 uMVPMatrix;
            uniform mat4 uModelMatrix;
            attribute vec4 aPosition;
            attribute vec3 aNormal;
            varying vec3 vNormal;
            varying vec3 vPosition;
            void main() {
                gl_Position = uMVPMatrix * aPosition;
                vNormal = mat3(uModelMatrix) * aNormal;
                vPosition = (uModelMatrix * aPosition).xyz;
            }
        """.trimIndent()

        val objectFS = """
            precision mediump float;
            uniform vec4 uColor;
            uniform float uConfidence;
            varying vec3 vNormal;
            varying vec3 vPosition;
            void main() {
                vec3 lightDir = normalize(vec3(0.5, 1.0, 0.5));
                vec3 normal = normalize(vNormal);
                float diff = max(dot(normal, lightDir), 0.0);
                float ambient = 0.3;
                vec3 color = uColor.rgb * (ambient + diff * 0.7);

                // Confidence affects alpha
                float alpha = uColor.a * (0.5 + uConfidence * 0.5);

                // Add slight rim lighting for depth
                vec3 viewDir = normalize(-vPosition);
                float rim = 1.0 - max(dot(viewDir, normal), 0.0);
                color += vec3(0.2) * pow(rim, 3.0);

                gl_FragColor = vec4(color, alpha);
            }
        """.trimIndent()

        objectProgram = createProgram(objectVS, objectFS)

        // Simple line shader for axes and planes
        val lineVS = """
            uniform mat4 uMVPMatrix;
            attribute vec4 aPosition;
            attribute vec4 aColor;
            varying vec4 vColor;
            void main() {
                gl_Position = uMVPMatrix * aPosition;
                vColor = aColor;
            }
        """.trimIndent()

        val lineFS = """
            precision mediump float;
            varying vec4 vColor;
            void main() {
                gl_FragColor = vColor;
            }
        """.trimIndent()

        lineProgram = createProgram(lineVS, lineFS)
    }

    private fun createProgram(vertexSource: String, fragmentSource: String): Int {
        val vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, vertexSource)
        val fragmentShader = loadShader(GLES20.GL_FRAGMENT_SHADER, fragmentSource)

        val program = GLES20.glCreateProgram()
        GLES20.glAttachShader(program, vertexShader)
        GLES20.glAttachShader(program, fragmentShader)
        GLES20.glLinkProgram(program)
        return program
    }

    private fun loadShader(type: Int, source: String): Int {
        val shader = GLES20.glCreateShader(type)
        GLES20.glShaderSource(shader, source)
        GLES20.glCompileShader(shader)
        return shader
    }

    private fun createGeometry() {
        // Unit cube centered at origin
        val cubeVerts = floatArrayOf(
            // Front face
            -0.5f, -0.5f,  0.5f,   0.5f, -0.5f,  0.5f,   0.5f,  0.5f,  0.5f,  -0.5f,  0.5f,  0.5f,
            // Back face
            -0.5f, -0.5f, -0.5f,  -0.5f,  0.5f, -0.5f,   0.5f,  0.5f, -0.5f,   0.5f, -0.5f, -0.5f,
            // Top face
            -0.5f,  0.5f, -0.5f,  -0.5f,  0.5f,  0.5f,   0.5f,  0.5f,  0.5f,   0.5f,  0.5f, -0.5f,
            // Bottom face
            -0.5f, -0.5f, -0.5f,   0.5f, -0.5f, -0.5f,   0.5f, -0.5f,  0.5f,  -0.5f, -0.5f,  0.5f,
            // Right face
             0.5f, -0.5f, -0.5f,   0.5f,  0.5f, -0.5f,   0.5f,  0.5f,  0.5f,   0.5f, -0.5f,  0.5f,
            // Left face
            -0.5f, -0.5f, -0.5f,  -0.5f, -0.5f,  0.5f,  -0.5f,  0.5f,  0.5f,  -0.5f,  0.5f, -0.5f
        )
        cubeVertices = createFloatBuffer(cubeVerts)

        val cubeNorms = floatArrayOf(
            // Front
            0f, 0f, 1f, 0f, 0f, 1f, 0f, 0f, 1f, 0f, 0f, 1f,
            // Back
            0f, 0f, -1f, 0f, 0f, -1f, 0f, 0f, -1f, 0f, 0f, -1f,
            // Top
            0f, 1f, 0f, 0f, 1f, 0f, 0f, 1f, 0f, 0f, 1f, 0f,
            // Bottom
            0f, -1f, 0f, 0f, -1f, 0f, 0f, -1f, 0f, 0f, -1f, 0f,
            // Right
            1f, 0f, 0f, 1f, 0f, 0f, 1f, 0f, 0f, 1f, 0f, 0f,
            // Left
            -1f, 0f, 0f, -1f, 0f, 0f, -1f, 0f, 0f, -1f, 0f, 0f
        )
        cubeNormals = createFloatBuffer(cubeNorms)

        val cubeIdx = shortArrayOf(
            0, 1, 2, 0, 2, 3,       // Front
            4, 5, 6, 4, 6, 7,       // Back
            8, 9, 10, 8, 10, 11,    // Top
            12, 13, 14, 12, 14, 15, // Bottom
            16, 17, 18, 16, 18, 19, // Right
            20, 21, 22, 20, 22, 23  // Left
        )
        cubeIndices = createShortBuffer(cubeIdx)

        // Generate UV sphere
        createSphere(16, 16)

        // Pyramid
        val pyramidVerts = floatArrayOf(
            // Base
            -0.5f, 0f, -0.5f,   0.5f, 0f, -0.5f,   0.5f, 0f,  0.5f,
            -0.5f, 0f, -0.5f,   0.5f, 0f,  0.5f,  -0.5f, 0f,  0.5f,
            // Front
            -0.5f, 0f,  0.5f,   0.5f, 0f,  0.5f,   0f, 1f, 0f,
            // Right
             0.5f, 0f,  0.5f,   0.5f, 0f, -0.5f,   0f, 1f, 0f,
            // Back
             0.5f, 0f, -0.5f,  -0.5f, 0f, -0.5f,   0f, 1f, 0f,
            // Left
            -0.5f, 0f, -0.5f,  -0.5f, 0f,  0.5f,   0f, 1f, 0f
        )
        pyramidVertices = createFloatBuffer(pyramidVerts)

        val pyramidNorms = floatArrayOf(
            // Base (down)
            0f, -1f, 0f, 0f, -1f, 0f, 0f, -1f, 0f,
            0f, -1f, 0f, 0f, -1f, 0f, 0f, -1f, 0f,
            // Front
            0f, 0.447f, 0.894f, 0f, 0.447f, 0.894f, 0f, 0.447f, 0.894f,
            // Right
            0.894f, 0.447f, 0f, 0.894f, 0.447f, 0f, 0.894f, 0.447f, 0f,
            // Back
            0f, 0.447f, -0.894f, 0f, 0.447f, -0.894f, 0f, 0.447f, -0.894f,
            // Left
            -0.894f, 0.447f, 0f, -0.894f, 0.447f, 0f, -0.894f, 0.447f, 0f
        )
        pyramidNormals = createFloatBuffer(pyramidNorms)

        // Plane grid
        val planeGrid = mutableListOf<Float>()
        val gridSize = 1f
        val gridStep = 0.1f
        var x = -gridSize
        while (x <= gridSize) {
            planeGrid.addAll(listOf(x, 0f, -gridSize, x, 0f, gridSize))
            planeGrid.addAll(listOf(-gridSize, 0f, x, gridSize, 0f, x))
            x += gridStep
        }
        planeVertices = createFloatBuffer(planeGrid.toFloatArray())

        // Coordinate axes
        val axesVerts = floatArrayOf(
            0f, 0f, 0f, 1f, 0f, 0f,  // X axis
            0f, 0f, 0f, 0f, 1f, 0f,  // Y axis
            0f, 0f, 0f, 0f, 0f, 1f   // Z axis
        )
        axesVertices = createFloatBuffer(axesVerts)

        val axesCols = floatArrayOf(
            1f, 0f, 0f, 1f, 1f, 0f, 0f, 1f,  // Red (X)
            0f, 1f, 0f, 1f, 0f, 1f, 0f, 1f,  // Green (Y)
            0f, 0f, 1f, 1f, 0f, 0f, 1f, 1f   // Blue (Z)
        )
        axesColors = createFloatBuffer(axesCols)
    }

    private fun createSphere(rings: Int, sectors: Int) {
        val vertices = mutableListOf<Float>()
        val normals = mutableListOf<Float>()
        val indices = mutableListOf<Short>()

        val R = 1f / (rings - 1)
        val S = 1f / (sectors - 1)

        for (r in 0 until rings) {
            for (s in 0 until sectors) {
                val y = sin(-Math.PI / 2 + Math.PI * r * R).toFloat()
                val x = (cos(2 * Math.PI * s * S) * sin(Math.PI * r * R)).toFloat()
                val z = (sin(2 * Math.PI * s * S) * sin(Math.PI * r * R)).toFloat()

                vertices.addAll(listOf(x * 0.5f, y * 0.5f, z * 0.5f))
                normals.addAll(listOf(x, y, z))
            }
        }

        for (r in 0 until rings - 1) {
            for (s in 0 until sectors - 1) {
                val curr = (r * sectors + s).toShort()
                val next = (curr + sectors).toShort()

                indices.add(curr)
                indices.add((curr + 1).toShort())
                indices.add(next)

                indices.add((curr + 1).toShort())
                indices.add((next + 1).toShort())
                indices.add(next)
            }
        }

        sphereVertices = createFloatBuffer(vertices.toFloatArray())
        sphereNormals = createFloatBuffer(normals.toFloatArray())
        sphereIndices = createShortBuffer(indices.toShortArray())
        sphereIndexCount = indices.size
    }

    fun setViewport(width: Int, height: Int) {
        val ratio = width.toFloat() / height
        Matrix.perspectiveM(projectionMatrix, 0, 60f, ratio, 0.1f, 100f)
    }

    fun setCameraPose(pose: Pose) {
        // Convert pose to view matrix (inverse of camera pose)
        val poseMatrix = pose.toMatrix()
        Matrix.invertM(viewMatrix, 0, poseMatrix, 0)
    }

    fun drawObject(obj: VirtualObject, anchorPose: Pose, confidence: Float) {
        GLES20.glUseProgram(objectProgram)
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)

        // Model matrix from anchor pose
        val anchorMatrix = anchorPose.toMatrix()
        Matrix.setIdentityM(modelMatrix, 0)
        Matrix.translateM(modelMatrix, 0, anchorPose.x, anchorPose.y, anchorPose.z)

        // Apply rotation from quaternion
        val rotMatrix = FloatArray(16)
        quaternionToMatrix(anchorPose.qw, anchorPose.qx, anchorPose.qy, anchorPose.qz, rotMatrix)
        Matrix.multiplyMM(tempMatrix, 0, modelMatrix, 0, rotMatrix, 0)
        System.arraycopy(tempMatrix, 0, modelMatrix, 0, 16)

        // Scale
        Matrix.scaleM(modelMatrix, 0, obj.scale, obj.scale, obj.scale)

        // MVP matrix
        Matrix.multiplyMM(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0)

        // Set uniforms
        val mvpHandle = GLES20.glGetUniformLocation(objectProgram, "uMVPMatrix")
        val modelHandle = GLES20.glGetUniformLocation(objectProgram, "uModelMatrix")
        val colorHandle = GLES20.glGetUniformLocation(objectProgram, "uColor")
        val confHandle = GLES20.glGetUniformLocation(objectProgram, "uConfidence")

        GLES20.glUniformMatrix4fv(mvpHandle, 1, false, mvpMatrix, 0)
        GLES20.glUniformMatrix4fv(modelHandle, 1, false, modelMatrix, 0)
        GLES20.glUniform4fv(colorHandle, 1, obj.color, 0)
        GLES20.glUniform1f(confHandle, confidence)

        // Draw based on type
        when (obj.type) {
            VirtualObjectType.CUBE -> drawCube()
            VirtualObjectType.SPHERE -> drawSphere()
            VirtualObjectType.PYRAMID -> drawPyramid()
            VirtualObjectType.CYLINDER -> drawCube() // Simplified
            VirtualObjectType.TORUS -> drawSphere() // Simplified
        }

        GLES20.glDisable(GLES20.GL_BLEND)
    }

    private fun drawCube() {
        val posHandle = GLES20.glGetAttribLocation(objectProgram, "aPosition")
        val normHandle = GLES20.glGetAttribLocation(objectProgram, "aNormal")

        GLES20.glEnableVertexAttribArray(posHandle)
        GLES20.glEnableVertexAttribArray(normHandle)

        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, 0, cubeVertices)
        GLES20.glVertexAttribPointer(normHandle, 3, GLES20.GL_FLOAT, false, 0, cubeNormals)

        GLES20.glDrawElements(GLES20.GL_TRIANGLES, 36, GLES20.GL_UNSIGNED_SHORT, cubeIndices)

        GLES20.glDisableVertexAttribArray(posHandle)
        GLES20.glDisableVertexAttribArray(normHandle)
    }

    private fun drawSphere() {
        val posHandle = GLES20.glGetAttribLocation(objectProgram, "aPosition")
        val normHandle = GLES20.glGetAttribLocation(objectProgram, "aNormal")

        GLES20.glEnableVertexAttribArray(posHandle)
        GLES20.glEnableVertexAttribArray(normHandle)

        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, 0, sphereVertices)
        GLES20.glVertexAttribPointer(normHandle, 3, GLES20.GL_FLOAT, false, 0, sphereNormals)

        GLES20.glDrawElements(GLES20.GL_TRIANGLES, sphereIndexCount, GLES20.GL_UNSIGNED_SHORT, sphereIndices)

        GLES20.glDisableVertexAttribArray(posHandle)
        GLES20.glDisableVertexAttribArray(normHandle)
    }

    private fun drawPyramid() {
        val posHandle = GLES20.glGetAttribLocation(objectProgram, "aPosition")
        val normHandle = GLES20.glGetAttribLocation(objectProgram, "aNormal")

        GLES20.glEnableVertexAttribArray(posHandle)
        GLES20.glEnableVertexAttribArray(normHandle)

        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, 0, pyramidVertices)
        GLES20.glVertexAttribPointer(normHandle, 3, GLES20.GL_FLOAT, false, 0, pyramidNormals)

        GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, 18)

        GLES20.glDisableVertexAttribArray(posHandle)
        GLES20.glDisableVertexAttribArray(normHandle)
    }

    fun drawPlane(plane: PlaneData) {
        GLES20.glUseProgram(lineProgram)
        GLES20.glLineWidth(1f)

        // Position plane at detected location
        Matrix.setIdentityM(modelMatrix, 0)
        Matrix.translateM(modelMatrix, 0, plane.centerX, plane.centerY, plane.centerZ)

        // Orient to plane normal
        if (!plane.isVertical) {
            // Horizontal plane - no rotation needed
        } else {
            // Vertical plane - rotate to align with normal
            val angle = Math.atan2(plane.normalX.toDouble(), plane.normalZ.toDouble()).toFloat()
            Matrix.rotateM(modelMatrix, 0, Math.toDegrees(angle.toDouble()).toFloat(), 0f, 1f, 0f)
        }

        // Scale to plane size
        Matrix.scaleM(modelMatrix, 0, plane.width, 1f, plane.height)

        // MVP
        Matrix.multiplyMM(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0)

        val mvpHandle = GLES20.glGetUniformLocation(lineProgram, "uMVPMatrix")
        GLES20.glUniformMatrix4fv(mvpHandle, 1, false, mvpMatrix, 0)

        val posHandle = GLES20.glGetAttribLocation(lineProgram, "aPosition")
        val colorHandle = GLES20.glGetAttribLocation(lineProgram, "aColor")

        GLES20.glEnableVertexAttribArray(posHandle)
        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, 0, planeVertices)

        // Set plane color (semi-transparent white)
        GLES20.glVertexAttrib4f(colorHandle, 1f, 1f, 1f, 0.3f)

        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)
        GLES20.glDrawArrays(GLES20.GL_LINES, 0, planeVertices.capacity() / 3)
        GLES20.glDisable(GLES20.GL_BLEND)

        GLES20.glDisableVertexAttribArray(posHandle)
    }

    fun drawAxes(x: Float, y: Float, z: Float, scale: Float) {
        GLES20.glUseProgram(lineProgram)
        GLES20.glLineWidth(3f)

        Matrix.setIdentityM(modelMatrix, 0)
        Matrix.translateM(modelMatrix, 0, x, y, z)
        Matrix.scaleM(modelMatrix, 0, scale, scale, scale)

        Matrix.multiplyMM(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0)

        val mvpHandle = GLES20.glGetUniformLocation(lineProgram, "uMVPMatrix")
        GLES20.glUniformMatrix4fv(mvpHandle, 1, false, mvpMatrix, 0)

        val posHandle = GLES20.glGetAttribLocation(lineProgram, "aPosition")
        val colorHandle = GLES20.glGetAttribLocation(lineProgram, "aColor")

        GLES20.glEnableVertexAttribArray(posHandle)
        GLES20.glEnableVertexAttribArray(colorHandle)

        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, 0, axesVertices)
        GLES20.glVertexAttribPointer(colorHandle, 4, GLES20.GL_FLOAT, false, 0, axesColors)

        GLES20.glDrawArrays(GLES20.GL_LINES, 0, 6)

        GLES20.glDisableVertexAttribArray(posHandle)
        GLES20.glDisableVertexAttribArray(colorHandle)
    }

    private fun quaternionToMatrix(w: Float, x: Float, y: Float, z: Float, m: FloatArray) {
        val xx = x * x
        val yy = y * y
        val zz = z * z
        val xy = x * y
        val xz = x * z
        val yz = y * z
        val wx = w * x
        val wy = w * y
        val wz = w * z

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

        m[12] = 0f
        m[13] = 0f
        m[14] = 0f
        m[15] = 1f
    }

    private fun createFloatBuffer(data: FloatArray): FloatBuffer {
        return ByteBuffer.allocateDirect(data.size * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .apply {
                put(data)
                position(0)
            }
    }

    private fun createShortBuffer(data: ShortArray): ShortBuffer {
        return ByteBuffer.allocateDirect(data.size * 2)
            .order(ByteOrder.nativeOrder())
            .asShortBuffer()
            .apply {
                put(data)
                position(0)
            }
    }
}
