#pragma once

// ============================================================
// HoloPhysics Lab 2 — Mahalanobis Outlier Rejection
//
// Решает проблему "галлюцинаций" MediaPipe/OpenCV:
//   палец телепортируется на 20 см на 1 кадр → выброс
//
// Решение: Расстояние Махаланобиса между предсказанием
//   и измерением. Если DM > χ²_threshold → reject.
//   Дополнительно: проверка инвариантов анатомии кисти.
// ============================================================

#include "../core/Types.h"
#include "../cv/HandTracker.h"
#include <cmath>
#include <array>
#include <deque>

namespace hlp {

struct MahalanobisConfig {
    // Chi-square threshold for 3 DOF at 99.7% confidence
    double chi_sq_threshold = 14.16;  // 3 DOF, p=0.997
    
    // Bone length invariance
    double max_bone_length_change = 0.05;  // 5% max change per frame
    
    // History for noise estimation
    int history_window = 10;
    
    // Auto-correction
    bool auto_correct_bones = true;  // Fix bone lengths on valid frames
    
    // Debug
    bool log_rejections = false;
};

struct BoneLengths {
    // 20 bones: 5 fingers × 4 bones each
    // Index: finger*4 + bone
    std::array<double, 20> lengths;
    bool initialized = false;
    
    void initialize(const HandData& hand);
    double compute_change(const HandData& hand) const;
    bool is_valid(const HandData& hand, double max_change) const;
};

class MahalanobisGater {
public:
    MahalanobisGater();
    ~MahalanobisGater() = default;
    
    // === Main gate ===
    // Returns true if measurement passes the gate
    bool gate(const dvec3& prediction, const dvec3& measurement,
              const double covariance[9],  // 3x3 innovation covariance S
              double& mahalanobis_dist);
    
    // === Bone length validation ===
    bool validate_anatomy(const HandData& hand);
    
    // === Adaptive threshold ===
    // Adjusts threshold based on recent rejection rate
    void update_threshold(double rejection_rate);
    
    // === Statistics ===
    int total_frames() const { return total_; }
    int rejected_frames() const { return rejected_; }
    double rejection_rate() const { return total_ > 0 ? (double)rejected_ / total_ : 0; }
    
    // === Configuration ===
    void set_config(const MahalanobisConfig& cfg) { config_ = cfg; }
    MahalanobisConfig& config() { return config_; }
    
    // === Reset ===
    void reset();
    
private:
    MahalanobisConfig config_;
    BoneLengths bone_lengths_;
    int total_ = 0;
    int rejected_ = 0;
    
    // History for adaptive noise
    std::deque<double> innovation_history_;
    
    // 3x3 matrix operations
    static double determinant_33(const double m[9]);
    static bool invert_33(const double m[9], double inv[9]);
};

} // namespace hlp
