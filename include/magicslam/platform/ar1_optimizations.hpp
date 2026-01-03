#pragma once

#include "../core/types.hpp"
#include <cstdint>
#include <cstring>

// Platform detection
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define HAS_NEON 1
#else
#define HAS_NEON 0
#endif

namespace MagicSLAM {
namespace Platform {

// Memory alignment for NEON (16 bytes)
constexpr size_t NEON_ALIGNMENT = 16;

// Aligned memory allocation
inline void* alignedAlloc(size_t size, size_t alignment = NEON_ALIGNMENT) {
    void* ptr = nullptr;
#if defined(_MSC_VER)
    ptr = _aligned_malloc(size, alignment);
#else
    if (posix_memalign(&ptr, alignment, size) != 0) {
        ptr = nullptr;
    }
#endif
    return ptr;
}

inline void alignedFree(void* ptr) {
#if defined(_MSC_VER)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

// NEON-optimized operations
namespace NEON {

#if HAS_NEON

// Fast 4-element dot product
inline float dot4(const float* a, const float* b) {
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t prod = vmulq_f32(va, vb);
    float32x2_t sum = vadd_f32(vget_low_f32(prod), vget_high_f32(prod));
    sum = vpadd_f32(sum, sum);
    return vget_lane_f32(sum, 0);
}

// Fast matrix-vector multiply (3x3 * 3)
inline void matVec3(const float* m, const float* v, float* result) {
    float32x4_t v0 = vld1q_f32(v);  // Load v (with extra element)

    float32x4_t row0 = vld1q_f32(m);
    float32x4_t row1 = vld1q_f32(m + 3);
    float32x4_t row2 = vld1q_f32(m + 6);

    float32x4_t prod0 = vmulq_f32(row0, v0);
    float32x4_t prod1 = vmulq_f32(row1, v0);
    float32x4_t prod2 = vmulq_f32(row2, v0);

    float32x2_t sum0 = vadd_f32(vget_low_f32(prod0), vget_high_f32(prod0));
    float32x2_t sum1 = vadd_f32(vget_low_f32(prod1), vget_high_f32(prod1));
    float32x2_t sum2 = vadd_f32(vget_low_f32(prod2), vget_high_f32(prod2));

    result[0] = vget_lane_f32(vpadd_f32(sum0, sum0), 0);
    result[1] = vget_lane_f32(vpadd_f32(sum1, sum1), 0);
    result[2] = vget_lane_f32(vpadd_f32(sum2, sum2), 0);
}

// Fast quaternion multiply
inline void quatMul(const float* a, const float* b, float* result) {
    float32x4_t qa = vld1q_f32(a);  // w, x, y, z
    float32x4_t qb = vld1q_f32(b);

    // Extract components
    float aw = vgetq_lane_f32(qa, 0);
    float ax = vgetq_lane_f32(qa, 1);
    float ay = vgetq_lane_f32(qa, 2);
    float az = vgetq_lane_f32(qa, 3);

    float bw = vgetq_lane_f32(qb, 0);
    float bx = vgetq_lane_f32(qb, 1);
    float by = vgetq_lane_f32(qb, 2);
    float bz = vgetq_lane_f32(qb, 3);

    result[0] = aw * bw - ax * bx - ay * by - az * bz;
    result[1] = aw * bx + ax * bw + ay * bz - az * by;
    result[2] = aw * by - ax * bz + ay * bw + az * bx;
    result[3] = aw * bz + ax * by - ay * bx + az * bw;
}

// Fast descriptor distance (Hamming) using NEON popcount
inline int descriptorDistance(const uint8_t* a, const uint8_t* b, int size) {
    int dist = 0;

    for (int i = 0; i < size; i += 8) {
        uint8x8_t va = vld1_u8(a + i);
        uint8x8_t vb = vld1_u8(b + i);
        uint8x8_t xored = veor_u8(va, vb);
        uint8x8_t popcnt = vcnt_u8(xored);

        // Sum popcount
        uint16x4_t sum16 = vpaddl_u8(popcnt);
        uint32x2_t sum32 = vpaddl_u16(sum16);
        uint64x1_t sum64 = vpaddl_u32(sum32);
        dist += vget_lane_u64(sum64, 0);
    }

    return dist;
}

// Fast image downsampling (2x2 box filter)
inline void downsample2x(const uint8_t* src, uint8_t* dst,
                        int srcWidth, int srcHeight) {
    int dstWidth = srcWidth / 2;
    int dstHeight = srcHeight / 2;

    for (int y = 0; y < dstHeight; ++y) {
        const uint8_t* row0 = src + (y * 2) * srcWidth;
        const uint8_t* row1 = row0 + srcWidth;
        uint8_t* dstRow = dst + y * dstWidth;

        int x = 0;
        for (; x + 8 <= dstWidth; x += 8) {
            // Load 16 pixels from each row
            uint8x16_t v0 = vld1q_u8(row0 + x * 2);
            uint8x16_t v1 = vld1q_u8(row1 + x * 2);

            // Pairwise add adjacent pixels
            uint16x8_t sum0 = vpaddlq_u8(v0);
            uint16x8_t sum1 = vpaddlq_u8(v1);

            // Add rows and divide by 4
            uint16x8_t sum = vaddq_u16(sum0, sum1);
            uint8x8_t result = vshrn_n_u16(sum, 2);

            vst1_u8(dstRow + x, result);
        }

        // Handle remaining pixels
        for (; x < dstWidth; ++x) {
            int sx = x * 2;
            int sum = row0[sx] + row0[sx + 1] + row1[sx] + row1[sx + 1];
            dstRow[x] = static_cast<uint8_t>(sum >> 2);
        }
    }
}

// Fast Gaussian blur 3x3
inline void gaussianBlur3x3(const uint8_t* src, uint8_t* dst,
                           int width, int height) {
    // Gaussian kernel: [1 2 1; 2 4 2; 1 2 1] / 16
    for (int y = 1; y < height - 1; ++y) {
        const uint8_t* row0 = src + (y - 1) * width;
        const uint8_t* row1 = src + y * width;
        const uint8_t* row2 = src + (y + 1) * width;
        uint8_t* dstRow = dst + y * width;

        int x = 1;
        for (; x + 8 <= width - 1; x += 8) {
            // Load 10 pixels from each row (for 8 output pixels)
            uint8x16_t v0 = vld1q_u8(row0 + x - 1);
            uint8x16_t v1 = vld1q_u8(row1 + x - 1);
            uint8x16_t v2 = vld1q_u8(row2 + x - 1);

            // Apply horizontal convolution with [1 2 1]
            uint16x8_t h0 = vaddl_u8(vget_low_u8(v0), vget_low_u8(vextq_u8(v0, v0, 2)));
            h0 = vaddq_u16(h0, vshll_n_u8(vget_low_u8(vextq_u8(v0, v0, 1)), 1));

            uint16x8_t h1 = vaddl_u8(vget_low_u8(v1), vget_low_u8(vextq_u8(v1, v1, 2)));
            h1 = vaddq_u16(h1, vshll_n_u8(vget_low_u8(vextq_u8(v1, v1, 1)), 1));

            uint16x8_t h2 = vaddl_u8(vget_low_u8(v2), vget_low_u8(vextq_u8(v2, v2, 2)));
            h2 = vaddq_u16(h2, vshll_n_u8(vget_low_u8(vextq_u8(v2, v2, 1)), 1));

            // Apply vertical convolution with [1 2 1] and divide by 16
            uint16x8_t sum = vaddq_u16(h0, h2);
            sum = vaddq_u16(sum, vshlq_n_u16(h1, 1));
            uint8x8_t result = vshrn_n_u16(sum, 4);

            vst1_u8(dstRow + x, result);
        }

        // Handle remaining pixels
        for (; x < width - 1; ++x) {
            int sum = row0[x-1] + 2*row0[x] + row0[x+1] +
                     2*row1[x-1] + 4*row1[x] + 2*row1[x+1] +
                     row2[x-1] + 2*row2[x] + row2[x+1];
            dstRow[x] = static_cast<uint8_t>(sum >> 4);
        }
    }

    // Copy border
    std::memcpy(dst, src, width);
    std::memcpy(dst + (height-1)*width, src + (height-1)*width, width);
    for (int y = 0; y < height; ++y) {
        dst[y * width] = src[y * width];
        dst[y * width + width - 1] = src[y * width + width - 1];
    }
}

#else  // No NEON - scalar fallbacks

inline float dot4(const float* a, const float* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

inline void matVec3(const float* m, const float* v, float* result) {
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
    result[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
    result[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

inline void quatMul(const float* a, const float* b, float* result) {
    result[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    result[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    result[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    result[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

inline int descriptorDistance(const uint8_t* a, const uint8_t* b, int size) {
    int dist = 0;
    for (int i = 0; i < size; ++i) {
        dist += __builtin_popcount(a[i] ^ b[i]);
    }
    return dist;
}

inline void downsample2x(const uint8_t* src, uint8_t* dst,
                        int srcWidth, int srcHeight) {
    int dstWidth = srcWidth / 2;
    int dstHeight = srcHeight / 2;

    for (int y = 0; y < dstHeight; ++y) {
        for (int x = 0; x < dstWidth; ++x) {
            int sx = x * 2;
            int sy = y * 2;
            int sum = src[sy * srcWidth + sx] +
                     src[sy * srcWidth + sx + 1] +
                     src[(sy + 1) * srcWidth + sx] +
                     src[(sy + 1) * srcWidth + sx + 1];
            dst[y * dstWidth + x] = static_cast<uint8_t>(sum >> 2);
        }
    }
}

inline void gaussianBlur3x3(const uint8_t* src, uint8_t* dst,
                           int width, int height) {
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int sum = src[(y-1)*width + x-1] + 2*src[(y-1)*width + x] + src[(y-1)*width + x+1] +
                     2*src[y*width + x-1] + 4*src[y*width + x] + 2*src[y*width + x+1] +
                     src[(y+1)*width + x-1] + 2*src[(y+1)*width + x] + src[(y+1)*width + x+1];
            dst[y * width + x] = static_cast<uint8_t>(sum >> 4);
        }
    }
    std::memcpy(dst, src, width);
    std::memcpy(dst + (height-1)*width, src + (height-1)*width, width);
    for (int y = 0; y < height; ++y) {
        dst[y * width] = src[y * width];
        dst[y * width + width - 1] = src[y * width + width - 1];
    }
}

#endif  // HAS_NEON

}  // namespace NEON

// Memory pool for reduced allocations (critical for embedded)
class MemoryPool {
public:
    explicit MemoryPool(size_t blockSize, size_t numBlocks)
        : blockSize_(blockSize), numBlocks_(numBlocks) {
        size_t totalSize = blockSize * numBlocks;
        memory_ = static_cast<uint8_t*>(alignedAlloc(totalSize, NEON_ALIGNMENT));
        freeList_.reserve(numBlocks);
        for (size_t i = 0; i < numBlocks; ++i) {
            freeList_.push_back(memory_ + i * blockSize);
        }
    }

    ~MemoryPool() {
        alignedFree(memory_);
    }

    void* allocate() {
        if (freeList_.empty()) return nullptr;
        void* ptr = freeList_.back();
        freeList_.pop_back();
        return ptr;
    }

    void deallocate(void* ptr) {
        if (ptr >= memory_ && ptr < memory_ + blockSize_ * numBlocks_) {
            freeList_.push_back(static_cast<uint8_t*>(ptr));
        }
    }

    size_t available() const { return freeList_.size(); }

private:
    uint8_t* memory_;
    size_t blockSize_;
    size_t numBlocks_;
    std::vector<uint8_t*> freeList_;
};

// Fixed-point math utilities for DSP operations
namespace FixedPoint {

using Q16 = int32_t;  // Q15.16 fixed point
using Q31 = int32_t;  // Q0.31 fixed point

constexpr int Q16_BITS = 16;
constexpr Q16 Q16_ONE = 1 << Q16_BITS;

inline Q16 floatToQ16(float f) {
    return static_cast<Q16>(f * Q16_ONE);
}

inline float q16ToFloat(Q16 q) {
    return static_cast<float>(q) / Q16_ONE;
}

inline Q16 mulQ16(Q16 a, Q16 b) {
    return static_cast<Q16>((static_cast<int64_t>(a) * b) >> Q16_BITS);
}

inline Q16 divQ16(Q16 a, Q16 b) {
    return static_cast<Q16>((static_cast<int64_t>(a) << Q16_BITS) / b);
}

// Fast inverse square root (for normalization)
inline float fastInvSqrt(float x) {
    float xhalf = 0.5f * x;
    int32_t i;
    std::memcpy(&i, &x, 4);
    i = 0x5f3759df - (i >> 1);
    std::memcpy(&x, &i, 4);
    x = x * (1.5f - xhalf * x * x);  // One Newton iteration
    return x;
}

// Fast atan2 approximation
inline float fastAtan2(float y, float x) {
    constexpr float PI = 3.14159265358979f;
    constexpr float PI_2 = PI / 2.0f;

    float abs_y = std::abs(y) + 1e-10f;
    float angle;

    if (x >= 0) {
        float r = (x - abs_y) / (x + abs_y);
        angle = 0.1963f * r * r * r - 0.9817f * r + PI / 4.0f;
    } else {
        float r = (x + abs_y) / (abs_y - x);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.0f * PI / 4.0f;
    }

    return y < 0 ? -angle : angle;
}

// Fast sin/cos using polynomial approximation
inline void fastSinCos(float x, float& sinVal, float& cosVal) {
    constexpr float PI = 3.14159265358979f;
    constexpr float TWO_PI = 2.0f * PI;

    // Normalize to [-PI, PI]
    x = std::fmod(x + PI, TWO_PI);
    if (x < 0) x += TWO_PI;
    x -= PI;

    // Polynomial coefficients
    float x2 = x * x;

    // sin(x) ≈ x - x³/6 + x⁵/120
    sinVal = x * (1.0f - x2 * (1.0f/6.0f - x2 * (1.0f/120.0f)));

    // cos(x) ≈ 1 - x²/2 + x⁴/24
    cosVal = 1.0f - x2 * (0.5f - x2 * (1.0f/24.0f));
}

}  // namespace FixedPoint

// Power management hints for AR1
class PowerManager {
public:
    enum class PerformanceMode {
        LOW_POWER,      // Reduce tracking rate, save power
        BALANCED,       // Normal operation
        HIGH_PERFORMANCE // Maximum tracking quality
    };

    static void setMode(PerformanceMode mode) {
        currentMode_ = mode;
    }

    static PerformanceMode getMode() { return currentMode_; }

    // Get recommended IMU rate for current mode
    static int getImuRate() {
        switch (currentMode_) {
            case PerformanceMode::LOW_POWER: return 100;      // 100 Hz
            case PerformanceMode::BALANCED: return 200;       // 200 Hz
            case PerformanceMode::HIGH_PERFORMANCE: return 400; // 400 Hz
        }
        return 200;
    }

    // Get recommended camera rate
    static int getCameraRate() {
        switch (currentMode_) {
            case PerformanceMode::LOW_POWER: return 15;
            case PerformanceMode::BALANCED: return 30;
            case PerformanceMode::HIGH_PERFORMANCE: return 60;
        }
        return 30;
    }

    // Get recommended number of features to track
    static int getMaxFeatures() {
        switch (currentMode_) {
            case PerformanceMode::LOW_POWER: return 300;
            case PerformanceMode::BALANCED: return 500;
            case PerformanceMode::HIGH_PERFORMANCE: return 1000;
        }
        return 500;
    }

private:
    static inline PerformanceMode currentMode_ = PerformanceMode::BALANCED;
};

// Thermal throttling handler
class ThermalManager {
public:
    enum class ThermalState {
        NORMAL,
        WARM,
        HOT,
        CRITICAL
    };

    static void updateTemperature(float tempCelsius) {
        temperature_ = tempCelsius;

        if (tempCelsius < 40.0f) {
            state_ = ThermalState::NORMAL;
        } else if (tempCelsius < 50.0f) {
            state_ = ThermalState::WARM;
        } else if (tempCelsius < 60.0f) {
            state_ = ThermalState::HOT;
        } else {
            state_ = ThermalState::CRITICAL;
        }

        // Auto-adjust power mode
        if (state_ == ThermalState::CRITICAL) {
            PowerManager::setMode(PowerManager::PerformanceMode::LOW_POWER);
        } else if (state_ == ThermalState::HOT) {
            PowerManager::setMode(PowerManager::PerformanceMode::BALANCED);
        }
    }

    static ThermalState getState() { return state_; }
    static float getTemperature() { return temperature_; }
    static bool shouldThrottle() { return state_ >= ThermalState::HOT; }

private:
    static inline ThermalState state_ = ThermalState::NORMAL;
    static inline float temperature_ = 25.0f;
};

}  // namespace Platform
}  // namespace MagicSLAM
