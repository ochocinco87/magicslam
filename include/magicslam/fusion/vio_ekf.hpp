#pragma once

#include "../core/types.hpp"
#include "../core/feature.hpp"
#include "../imu/imu_integrator.hpp"
#include <array>
#include <cmath>

namespace MagicSLAM {

// State vector indices for 15-state EKF
// [position(3), velocity(3), orientation(3), acc_bias(3), gyro_bias(3)]
enum StateIndex {
    POS_X = 0, POS_Y = 1, POS_Z = 2,
    VEL_X = 3, VEL_Y = 4, VEL_Z = 5,
    ORI_X = 6, ORI_Y = 7, ORI_Z = 8,  // Rotation vector representation
    BA_X = 9, BA_Y = 10, BA_Z = 11,
    BG_X = 12, BG_Y = 13, BG_Z = 14,
    STATE_DIM = 15
};

// Compact matrix class for EKF operations (optimized for embedded)
template<int ROWS, int COLS>
class Matrix {
public:
    std::array<float, ROWS * COLS> data;

    Matrix() { data.fill(0); }

    float& operator()(int r, int c) { return data[r * COLS + c]; }
    float operator()(int r, int c) const { return data[r * COLS + c]; }

    static Matrix identity() {
        Matrix m;
        for (int i = 0; i < std::min(ROWS, COLS); ++i) {
            m(i, i) = 1.0f;
        }
        return m;
    }

    Matrix<ROWS, COLS> operator+(const Matrix<ROWS, COLS>& o) const {
        Matrix<ROWS, COLS> r;
        for (int i = 0; i < ROWS * COLS; ++i) {
            r.data[i] = data[i] + o.data[i];
        }
        return r;
    }

    Matrix<ROWS, COLS> operator-(const Matrix<ROWS, COLS>& o) const {
        Matrix<ROWS, COLS> r;
        for (int i = 0; i < ROWS * COLS; ++i) {
            r.data[i] = data[i] - o.data[i];
        }
        return r;
    }

    template<int COLS2>
    Matrix<ROWS, COLS2> operator*(const Matrix<COLS, COLS2>& o) const {
        Matrix<ROWS, COLS2> r;
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < COLS2; ++j) {
                float sum = 0;
                for (int k = 0; k < COLS; ++k) {
                    sum += (*this)(i, k) * o(k, j);
                }
                r(i, j) = sum;
            }
        }
        return r;
    }

    Matrix<COLS, ROWS> transpose() const {
        Matrix<COLS, ROWS> r;
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < COLS; ++j) {
                r(j, i) = (*this)(i, j);
            }
        }
        return r;
    }
};

// Symmetric matrix operations for covariance (stores only upper triangle)
template<int N>
class SymmetricMatrix {
public:
    std::array<float, N * (N + 1) / 2> data;

    SymmetricMatrix() { data.fill(0); }

    float get(int r, int c) const {
        if (r > c) std::swap(r, c);
        return data[r * N - r * (r - 1) / 2 + c - r];
    }

    void set(int r, int c, float val) {
        if (r > c) std::swap(r, c);
        data[r * N - r * (r - 1) / 2 + c - r] = val;
    }

    void addDiagonal(float val) {
        for (int i = 0; i < N; ++i) {
            set(i, i, get(i, i) + val);
        }
    }

    static SymmetricMatrix identity() {
        SymmetricMatrix m;
        for (int i = 0; i < N; ++i) {
            m.set(i, i, 1.0f);
        }
        return m;
    }

    // Convert to full matrix
    Matrix<N, N> toMatrix() const {
        Matrix<N, N> m;
        for (int i = 0; i < N; ++i) {
            for (int j = i; j < N; ++j) {
                float val = get(i, j);
                m(i, j) = val;
                m(j, i) = val;
            }
        }
        return m;
    }

    // Update from P = F*P*F' + Q (common EKF operation)
    void propagate(const Matrix<N, N>& F, const Matrix<N, N>& Q) {
        // P = F * P * F' + Q
        Matrix<N, N> P = toMatrix();
        Matrix<N, N> FP = F * P;
        Matrix<N, N> FPFT = FP * F.transpose();

        for (int i = 0; i < N; ++i) {
            for (int j = i; j < N; ++j) {
                set(i, j, FPFT(i, j) + Q(i, j));
            }
        }
    }
};

// Visual-Inertial Odometry Extended Kalman Filter
class VioEkf {
public:
    struct Config {
        // Process noise
        float accelNoise = 0.01f;        // m/s^2/sqrt(Hz)
        float gyroNoise = 0.001f;        // rad/s/sqrt(Hz)
        float accelBiasNoise = 0.0001f;  // m/s^2/sqrt(Hz)
        float gyroBiasNoise = 0.00001f;  // rad/s/sqrt(Hz)

        // Measurement noise
        float visualPositionNoise = 0.01f;   // m
        float visualOrientationNoise = 0.01f; // rad

        // Initial uncertainty
        float initialPositionStd = 0.1f;
        float initialVelocityStd = 0.1f;
        float initialOrientationStd = 0.1f;
        float initialAccBiasStd = 0.01f;
        float initialGyroBiasStd = 0.001f;

        Vec3 gravity{0, 0, -9.81f};
    };

    explicit VioEkf(const Config& config = {}) : config_(config) {
        reset();
    }

    void reset() {
        state_.pose = Pose{};
        state_.velocity = Vec3{};
        state_.timestamp = 0;
        bias_ = ImuBias{};

        // Initialize covariance
        covariance_ = SymmetricMatrix<STATE_DIM>{};
        covariance_.set(POS_X, POS_X, config_.initialPositionStd * config_.initialPositionStd);
        covariance_.set(POS_Y, POS_Y, config_.initialPositionStd * config_.initialPositionStd);
        covariance_.set(POS_Z, POS_Z, config_.initialPositionStd * config_.initialPositionStd);
        covariance_.set(VEL_X, VEL_X, config_.initialVelocityStd * config_.initialVelocityStd);
        covariance_.set(VEL_Y, VEL_Y, config_.initialVelocityStd * config_.initialVelocityStd);
        covariance_.set(VEL_Z, VEL_Z, config_.initialVelocityStd * config_.initialVelocityStd);
        covariance_.set(ORI_X, ORI_X, config_.initialOrientationStd * config_.initialOrientationStd);
        covariance_.set(ORI_Y, ORI_Y, config_.initialOrientationStd * config_.initialOrientationStd);
        covariance_.set(ORI_Z, ORI_Z, config_.initialOrientationStd * config_.initialOrientationStd);
        covariance_.set(BA_X, BA_X, config_.initialAccBiasStd * config_.initialAccBiasStd);
        covariance_.set(BA_Y, BA_Y, config_.initialAccBiasStd * config_.initialAccBiasStd);
        covariance_.set(BA_Z, BA_Z, config_.initialAccBiasStd * config_.initialAccBiasStd);
        covariance_.set(BG_X, BG_X, config_.initialGyroBiasStd * config_.initialGyroBiasStd);
        covariance_.set(BG_Y, BG_Y, config_.initialGyroBiasStd * config_.initialGyroBiasStd);
        covariance_.set(BG_Z, BG_Z, config_.initialGyroBiasStd * config_.initialGyroBiasStd);

        initialized_ = false;
    }

    // IMU prediction step
    void predictIMU(const ImuMeasurement& imu) {
        if (!initialized_) {
            state_.timestamp = imu.timestamp;
            initialized_ = true;
            lastImu_ = imu;
            return;
        }

        float dt = (imu.timestamp - state_.timestamp) * 1e-9f;
        if (dt <= 0 || dt > 0.5f) {
            lastImu_ = imu;
            return;
        }

        // Remove bias
        Vec3 acc = imu.acceleration - bias_.accelerometer;
        Vec3 gyro = imu.angularVelocity - bias_.gyroscope;

        // Current rotation matrix
        Mat3 R = Mat3::fromQuaternion(state_.pose.orientation);

        // State prediction
        // Position: p = p + v*dt + 0.5*R*a*dt^2 + 0.5*g*dt^2
        Vec3 accWorld = R * acc;
        state_.pose.position = state_.pose.position +
            state_.velocity * dt +
            (accWorld + config_.gravity) * (0.5f * dt * dt);

        // Velocity: v = v + R*a*dt + g*dt
        state_.velocity = state_.velocity + (accWorld + config_.gravity) * dt;

        // Orientation: q = q * exp(w*dt)
        Vec3 angleIncrement = gyro * dt;
        Quaternion dq = Quaternion::fromRotationVector(angleIncrement);
        state_.pose.orientation = state_.pose.orientation * dq;
        state_.pose.orientation = state_.pose.orientation.normalized();

        state_.angularVelocity = gyro;
        state_.timestamp = imu.timestamp;

        // Covariance propagation
        propagateCovariance(acc, gyro, dt, R);

        lastImu_ = imu;
    }

    // Visual update step
    void updateVisual(const Pose& visualPose, Timestamp timestamp,
                     float positionConfidence = 1.0f,
                     float orientationConfidence = 1.0f) {
        if (!initialized_) return;

        // Measurement innovation
        Vec3 posInnovation = visualPose.position - state_.pose.position;
        Vec3 oriInnovation = (visualPose.orientation *
            state_.pose.orientation.inverse()).toRotationVector();

        // Measurement Jacobian (identity for direct pose measurement)
        Matrix<6, STATE_DIM> H;
        H(0, POS_X) = 1; H(1, POS_Y) = 1; H(2, POS_Z) = 1;
        H(3, ORI_X) = 1; H(4, ORI_Y) = 1; H(5, ORI_Z) = 1;

        // Measurement noise
        Matrix<6, 6> R_meas;
        float posVar = config_.visualPositionNoise * config_.visualPositionNoise / positionConfidence;
        float oriVar = config_.visualOrientationNoise * config_.visualOrientationNoise / orientationConfidence;
        R_meas(0, 0) = posVar; R_meas(1, 1) = posVar; R_meas(2, 2) = posVar;
        R_meas(3, 3) = oriVar; R_meas(4, 4) = oriVar; R_meas(5, 5) = oriVar;

        // Kalman gain: K = P * H' * (H * P * H' + R)^-1
        Matrix<STATE_DIM, STATE_DIM> P = covariance_.toMatrix();
        Matrix<STATE_DIM, 6> PHt = P * H.transpose();
        Matrix<6, 6> S = H * PHt + R_meas;

        // Invert S (6x6) using simple Gaussian elimination
        Matrix<6, 6> S_inv = invert6x6(S);
        Matrix<STATE_DIM, 6> K = PHt * S_inv;

        // State update
        std::array<float, 6> innovation = {
            posInnovation.x, posInnovation.y, posInnovation.z,
            oriInnovation.x, oriInnovation.y, oriInnovation.z
        };

        std::array<float, STATE_DIM> dx;
        dx.fill(0);
        for (int i = 0; i < STATE_DIM; ++i) {
            for (int j = 0; j < 6; ++j) {
                dx[i] += K(i, j) * innovation[j];
            }
        }

        // Apply state correction
        state_.pose.position.x += dx[POS_X];
        state_.pose.position.y += dx[POS_Y];
        state_.pose.position.z += dx[POS_Z];
        state_.velocity.x += dx[VEL_X];
        state_.velocity.y += dx[VEL_Y];
        state_.velocity.z += dx[VEL_Z];

        Vec3 oriCorrection{dx[ORI_X], dx[ORI_Y], dx[ORI_Z]};
        Quaternion dq = Quaternion::fromRotationVector(oriCorrection);
        state_.pose.orientation = dq * state_.pose.orientation;
        state_.pose.orientation = state_.pose.orientation.normalized();

        bias_.accelerometer.x += dx[BA_X];
        bias_.accelerometer.y += dx[BA_Y];
        bias_.accelerometer.z += dx[BA_Z];
        bias_.gyroscope.x += dx[BG_X];
        bias_.gyroscope.y += dx[BG_Y];
        bias_.gyroscope.z += dx[BG_Z];

        // Covariance update: P = (I - K*H) * P
        Matrix<STATE_DIM, STATE_DIM> I = Matrix<STATE_DIM, STATE_DIM>::identity();
        Matrix<STATE_DIM, STATE_DIM> KH = K * H;
        Matrix<STATE_DIM, STATE_DIM> IKH = I - KH;
        Matrix<STATE_DIM, STATE_DIM> P_new = IKH * P;

        // Symmetrize and store
        for (int i = 0; i < STATE_DIM; ++i) {
            for (int j = i; j < STATE_DIM; ++j) {
                covariance_.set(i, j, 0.5f * (P_new(i, j) + P_new(j, i)));
            }
        }
    }

    // Update with map point observations
    void updateMapPoints(const std::vector<std::pair<Point2D, Vec3>>& observations,
                        const CameraIntrinsics& camera) {
        if (!initialized_ || observations.empty()) return;

        // Stack all observations
        int numObs = static_cast<int>(observations.size());
        int measDim = numObs * 2;

        // Simplified update: average the reprojection-based pose correction
        Vec3 posCorrection{};
        int validCount = 0;

        for (const auto& obs : observations) {
            // Transform point to camera frame
            Vec3 pCam = state_.pose.inverseTransformPoint(obs.second);
            if (pCam.z <= 0.1f) continue;

            // Project to image
            Point2D proj = camera.project(pCam);

            // Reprojection error
            float errX = obs.first.x - proj.x;
            float errY = obs.first.y - proj.y;

            // Back-project error to 3D correction (approximate)
            float depth = toFloat(pCam.z);
            Vec3 correction{
                errX * depth / camera.fx * 0.01f,
                errY * depth / camera.fy * 0.01f,
                0
            };

            // Transform to world frame
            posCorrection += state_.pose.orientation.rotate(correction);
            validCount++;
        }

        if (validCount > 0) {
            float scale = 1.0f / validCount;
            state_.pose.position += posCorrection * scale;
        }
    }

    // Getters
    const MotionState& getState() const { return state_; }
    const ImuBias& getBias() const { return bias_; }
    bool isInitialized() const { return initialized_; }

    // Get position uncertainty (sqrt of diagonal)
    Vec3 getPositionStd() const {
        return Vec3{
            std::sqrt(covariance_.get(POS_X, POS_X)),
            std::sqrt(covariance_.get(POS_Y, POS_Y)),
            std::sqrt(covariance_.get(POS_Z, POS_Z))
        };
    }

    // Set initial state
    void initialize(const Pose& pose, const Vec3& velocity, Timestamp timestamp) {
        state_.pose = pose;
        state_.velocity = velocity;
        state_.timestamp = timestamp;
        initialized_ = true;
    }

private:
    Config config_;
    MotionState state_;
    ImuBias bias_;
    SymmetricMatrix<STATE_DIM> covariance_;
    ImuMeasurement lastImu_;
    bool initialized_ = false;

    void propagateCovariance(const Vec3& acc, const Vec3& gyro, float dt, const Mat3& R) {
        // State transition matrix F
        Matrix<STATE_DIM, STATE_DIM> F = Matrix<STATE_DIM, STATE_DIM>::identity();

        // Position derivatives
        F(POS_X, VEL_X) = dt;
        F(POS_Y, VEL_Y) = dt;
        F(POS_Z, VEL_Z) = dt;

        // Velocity derivatives w.r.t. orientation (simplified)
        float dt2 = dt * dt * 0.5f;
        // Skew-symmetric matrix of R*a (for orientation effect on position)
        Vec3 Ra = R * acc;
        F(POS_X, ORI_Y) = Ra.z * dt2;
        F(POS_X, ORI_Z) = -Ra.y * dt2;
        F(POS_Y, ORI_X) = -Ra.z * dt2;
        F(POS_Y, ORI_Z) = Ra.x * dt2;
        F(POS_Z, ORI_X) = Ra.y * dt2;
        F(POS_Z, ORI_Y) = -Ra.x * dt2;

        F(VEL_X, ORI_Y) = Ra.z * dt;
        F(VEL_X, ORI_Z) = -Ra.y * dt;
        F(VEL_Y, ORI_X) = -Ra.z * dt;
        F(VEL_Y, ORI_Z) = Ra.x * dt;
        F(VEL_Z, ORI_X) = Ra.y * dt;
        F(VEL_Z, ORI_Y) = -Ra.x * dt;

        // Bias effects
        F(VEL_X, BA_X) = -R(0, 0) * dt;
        F(VEL_X, BA_Y) = -R(0, 1) * dt;
        F(VEL_X, BA_Z) = -R(0, 2) * dt;
        F(VEL_Y, BA_X) = -R(1, 0) * dt;
        F(VEL_Y, BA_Y) = -R(1, 1) * dt;
        F(VEL_Y, BA_Z) = -R(1, 2) * dt;
        F(VEL_Z, BA_X) = -R(2, 0) * dt;
        F(VEL_Z, BA_Y) = -R(2, 1) * dt;
        F(VEL_Z, BA_Z) = -R(2, 2) * dt;

        F(ORI_X, BG_X) = -dt;
        F(ORI_Y, BG_Y) = -dt;
        F(ORI_Z, BG_Z) = -dt;

        // Process noise Q
        Matrix<STATE_DIM, STATE_DIM> Q;
        float accVar = config_.accelNoise * config_.accelNoise * dt;
        float gyroVar = config_.gyroNoise * config_.gyroNoise * dt;
        float accBiasVar = config_.accelBiasNoise * config_.accelBiasNoise * dt;
        float gyroBiasVar = config_.gyroBiasNoise * config_.gyroBiasNoise * dt;

        // Position noise from acceleration
        Q(POS_X, POS_X) = accVar * dt * dt * 0.25f;
        Q(POS_Y, POS_Y) = accVar * dt * dt * 0.25f;
        Q(POS_Z, POS_Z) = accVar * dt * dt * 0.25f;

        Q(VEL_X, VEL_X) = accVar;
        Q(VEL_Y, VEL_Y) = accVar;
        Q(VEL_Z, VEL_Z) = accVar;

        Q(ORI_X, ORI_X) = gyroVar;
        Q(ORI_Y, ORI_Y) = gyroVar;
        Q(ORI_Z, ORI_Z) = gyroVar;

        Q(BA_X, BA_X) = accBiasVar;
        Q(BA_Y, BA_Y) = accBiasVar;
        Q(BA_Z, BA_Z) = accBiasVar;

        Q(BG_X, BG_X) = gyroBiasVar;
        Q(BG_Y, BG_Y) = gyroBiasVar;
        Q(BG_Z, BG_Z) = gyroBiasVar;

        // Propagate covariance
        covariance_.propagate(F, Q);
    }

    // Simple 6x6 matrix inversion using Gaussian elimination
    Matrix<6, 6> invert6x6(const Matrix<6, 6>& m) {
        Matrix<6, 6> aug;
        Matrix<6, 6> result = Matrix<6, 6>::identity();

        // Copy m to aug
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                aug(i, j) = m(i, j);
            }
        }

        // Gaussian elimination with partial pivoting
        for (int i = 0; i < 6; ++i) {
            // Find pivot
            int maxRow = i;
            float maxVal = std::abs(aug(i, i));
            for (int k = i + 1; k < 6; ++k) {
                if (std::abs(aug(k, i)) > maxVal) {
                    maxVal = std::abs(aug(k, i));
                    maxRow = k;
                }
            }

            // Swap rows
            if (maxRow != i) {
                for (int j = 0; j < 6; ++j) {
                    std::swap(aug(i, j), aug(maxRow, j));
                    std::swap(result(i, j), result(maxRow, j));
                }
            }

            // Scale pivot row
            float pivot = aug(i, i);
            if (std::abs(pivot) < 1e-10f) {
                pivot = 1e-10f;  // Regularization
            }
            for (int j = 0; j < 6; ++j) {
                aug(i, j) /= pivot;
                result(i, j) /= pivot;
            }

            // Eliminate column
            for (int k = 0; k < 6; ++k) {
                if (k != i) {
                    float factor = aug(k, i);
                    for (int j = 0; j < 6; ++j) {
                        aug(k, j) -= factor * aug(i, j);
                        result(k, j) -= factor * result(i, j);
                    }
                }
            }
        }

        return result;
    }
};

}  // namespace MagicSLAM
