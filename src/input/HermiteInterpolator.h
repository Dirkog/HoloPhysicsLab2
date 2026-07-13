#pragma once

// ============================================================
// HoloPhysics Lab 2 — Lock-Free Hermite Interpolator
//
// Решает проблему: CV-поток (30-60 Гц) → Physics (240 Гц)
//   Ступенчатая подача данных 30 Гц в физику 240 Гц
//   вызывает "лесенку ускорения" (force stuttering).
//
// Решение: Lock-Free Triple Ring Buffer + Hermite Spline.
//   Физика интерполирует между двумя последними снэпшотами
//   без единого мьютекса (zero mutex contention).
// ============================================================

#include "../core/Types.h"
#include <atomic>
#include <array>
#include <cstring>

namespace hlp {

// ==================== Hand Snapshot ====================

struct HandSnapshot {
    alignas(64) double palm_x = 0;     // Cache-line aligned
    double palm_y = 0;
    double palm_z = 0;
    double vel_x = 0;
    double vel_y = 0;
    double vel_z = 0;
    double confidence = 0;
    int64_t timestamp_us = 0;
    int frame_id = 0;
    bool valid = false;
    
    // 21 landmarks (for full hand state)
    double landmarks[21 * 3] = {0};
    
    void set_position(const dvec3& pos) {
        palm_x = pos.x; palm_y = pos.y; palm_z = pos.z;
    }
    
    dvec3 position() const { return dvec3(palm_x, palm_y, palm_z); }
    
    void set_velocity(const dvec3& vel) {
        vel_x = vel.x; vel_y = vel.y; vel_z = vel.z;
    }
    
    dvec3 velocity() const { return dvec3(vel_x, vel_y, vel_z); }
};

// ==================== Lock-Free Triple Ring Buffer ====================

class HandRingBuffer {
public:
    HandRingBuffer();
    ~HandRingBuffer() = default;

    // === Producer (CV thread) ===
    // Запись нового снэпшота (lock-free)
    void produce(const HandSnapshot& snapshot);
    
    // === Consumer (Physics/Render thread) ===
    // Получение двух последних снэпшотов для интерполяции
    struct SnapshotPair {
        HandSnapshot prev;
        HandSnapshot current;
        double dt;           // Time between snapshots
        bool has_two = false; // At least 2 snapshots available
    };
    
    SnapshotPair consume();
    
    // === Utility ===
    int write_count() const { return write_count_.load(); }
    int read_count() const { return read_count_.load(); }
    bool is_ready() const { return write_count_.load() >= 2; }

private:
    // Triple buffer slots (cache-line padded)
    static constexpr int NUM_SLOTS = 3;
    alignas(64) HandSnapshot slots_[NUM_SLOTS];
    
    // Atomic indices
    std::atomic<int> write_idx_{0};
    std::atomic<int> read_idx_{0};
    std::atomic<int> available_idx_{-1};
    
    // Counters for diagnostics
    std::atomic<int> write_count_{0};
    std::atomic<int> read_count_{0};
    
    // Last consumed stamps for dt calculation
    HandSnapshot last_consumed_;
    bool has_last_ = false;
};

// ==================== Hermite Spline Interpolator ====================

class HermiteInterpolator {
public:
    HermiteInterpolator() = default;
    
    // === Initialize with ring buffer ===
    void set_buffer(HandRingBuffer* buffer) { buffer_ = buffer; }
    
    // === Get interpolated hand state at time t_local ===
    // t_local = current physics time within the CV frame interval [0, dt]
    struct InterpolatedState {
        dvec3 position;
        dvec3 velocity;
        bool valid = false;
    };
    
    InterpolatedState sample(double t_local);
    
    // === Get latest raw snapshot (for initialization) ===
    dvec3 latest_position() const;

    // === Cubic Hermite basis functions ===
    static double h00(double t) { return 2*t*t*t - 3*t*t + 1; }   // p0 influence
    static double h10(double t) { return t*t*t - 2*t*t + t; }     // m0 influence
    static double h01(double t) { return -2*t*t*t + 3*t*t; }      // p1 influence
    static double h11(double t) { return t*t*t - t*t; }            // m1 influence
    
    // === Hermite interpolation of single component ===
    static double hermite(double p0, double m0, double p1, double m1, double t) {
        return h00(t)*p0 + h10(t)*m0 + h01(t)*p1 + h11(t)*m1;
    }
    
    // === Hermite interpolation of 3D vector ===
    static dvec3 hermite_vec(const dvec3& p0, const dvec3& m0,
                              const dvec3& p1, const dvec3& m1, double t) {
        return dvec3(
            hermite(p0.x, m0.x, p1.x, m1.x, t),
            hermite(p0.y, m0.y, p1.y, m1.y, t),
            hermite(p0.z, m0.z, p1.z, m1.z, t)
        );
    }
    
    // === Velocity from derivative of Hermite ===
    static dvec3 hermite_vel(const dvec3& p0, const dvec3& m0,
                              const dvec3& p1, const dvec3& m1, double t, double dt) {
        // dh00/dt = 6t² - 6t
        // dh10/dt = 3t² - 4t + 1
        // dh01/dt = -6t² + 6t
        // dh11/dt = 3t² - 2t
        
        double dh00 = 6*t*t - 6*t;
        double dh10 = 3*t*t - 4*t + 1;
        double dh01 = -6*t*t + 6*t;
        double dh11 = 3*t*t - 2*t;
        
        return dvec3(
            (dh00*p0.x + dh10*m0.x + dh01*p1.x + dh11*m1.x) / dt,
            (dh00*p0.y + dh10*m0.y + dh01*p1.y + dh11*m1.y) / dt,
            (dh00*p0.z + dh10*m0.z + dh01*p1.z + dh11*m1.z) / dt
        );
    }

private:
    HandRingBuffer* buffer_ = nullptr;
};

} // namespace hlp
