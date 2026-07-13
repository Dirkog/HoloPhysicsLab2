#include "MahalanobisGater.h"
#include <algorithm>
#include <iostream>

namespace hlp {

// ==================== BoneLengths ====================

void BoneLengths::initialize(const HandData& hand) {
    // 5 fingers × 4 bones each
    // Finger segments: CMC→MCP, MCP→PIP, PIP→DIP, DIP→TIP
    const int finger_starts[5] = {1, 5, 9, 13, 17};  // Thumb CMC, Index MCP, etc.
    
    for (int f = 0; f < 5; f++) {
        int base = finger_starts[f];
        for (int b = 0; b < 4; b++) {
            int a_idx = base + b;
            int b_idx = base + b + 1;
            if (a_idx < 21 && b_idx < 21) {
                float dx = hand[b_idx].x - hand[a_idx].x;
                float dy = hand[b_idx].y - hand[a_idx].y;
                float dz = (hand[b_idx].z - hand[a_idx].z) * 100;
                lengths[f * 4 + b] = std::sqrt(dx*dx + dy*dy + dz*dz);
            }
        }
    }
    initialized = true;
}

double BoneLengths::compute_change(const HandData& hand) const {
    if (!initialized) return 0;
    
    double max_change = 0;
    const int finger_starts[5] = {1, 5, 9, 13, 17};
    
    for (int f = 0; f < 5; f++) {
        int base = finger_starts[f];
        for (int b = 0; b < 4; b++) {
            int a_idx = base + b;
            int b_idx = base + b + 1;
            if (a_idx < 21 && b_idx < 21) {
                float dx = hand[b_idx].x - hand[a_idx].x;
                float dy = hand[b_idx].y - hand[a_idx].y;
                float dz = (hand[b_idx].z - hand[a_idx].z) * 100;
                double new_len = std::sqrt(dx*dx + dy*dy + dz*dz);
                double old_len = lengths[f * 4 + b];
                if (old_len > 0.001) {
                    double change = std::abs(new_len - old_len) / old_len;
                    max_change = std::max(max_change, change);
                }
            }
        }
    }
    return max_change;
}

bool BoneLengths::is_valid(const HandData& hand, double max_change) const {
    if (!initialized) return false;
    return compute_change(hand) < max_change;
}

// ==================== MahalanobisGater ====================

MahalanobisGater::MahalanobisGater() = default;

double MahalanobisGater::determinant_33(const double m[9]) {
    return m[0] * (m[4]*m[8] - m[5]*m[7])
         - m[3] * (m[1]*m[8] - m[2]*m[7])
         + m[6] * (m[1]*m[5] - m[2]*m[4]);
}

bool MahalanobisGater::invert_33(const double m[9], double inv[9]) {
    double det = determinant_33(m);
    if (std::abs(det) < 1e-20) return false;
    
    double inv_det = 1.0 / det;
    inv[0] = (m[4]*m[8] - m[5]*m[7]) * inv_det;
    inv[1] = (m[2]*m[7] - m[1]*m[8]) * inv_det;
    inv[2] = (m[3]*m[5] - m[2]*m[4]) * inv_det;
    inv[3] = (m[5]*m[6] - m[3]*m[8]) * inv_det;
    inv[4] = (m[0]*m[8] - m[2]*m[6]) * inv_det;
    inv[5] = (m[2]*m[3] - m[0]*m[5]) * inv_det;
    inv[6] = (m[3]*m[7] - m[4]*m[6]) * inv_det;
    inv[7] = (m[2]*m[6] - m[0]*m[7]) * inv_det;
    inv[8] = (m[0]*m[4] - m[1]*m[3]) * inv_det;
    
    return true;
}

bool MahalanobisGater::gate(const dvec3& prediction, const dvec3& measurement,
                             const double covariance[9],
                             double& mahalanobis_dist) {
    total_++;
    
    // Innovation vector (residual)
    double innov[3] = {
        measurement.x - prediction.x,
        measurement.y - prediction.y,
        measurement.z - prediction.z
    };
    
    // Compute Mahalanobis distance: D² = innovᵀ · S⁻¹ · innov
    double inv_S[9];
    if (!invert_33(covariance, inv_S)) {
        // Singular matrix → can't compute, pass through
        return true;
    }
    
    // innovᵀ · S⁻¹
    double temp[3] = {
        innov[0]*inv_S[0] + innov[1]*inv_S[3] + innov[2]*inv_S[6],
        innov[0]*inv_S[1] + innov[1]*inv_S[4] + innov[2]*inv_S[7],
        innov[0]*inv_S[2] + innov[1]*inv_S[5] + innov[2]*inv_S[8]
    };
    
    // D² = (innovᵀ · S⁻¹) · innov
    double D_sq = temp[0]*innov[0] + temp[1]*innov[1] + temp[2]*innov[2];
    mahalanobis_dist = std::sqrt(D_sq);
    
    // Gate: reject if Mahalanobis distance exceeds threshold
    if (D_sq > config_.chi_sq_threshold) {
        rejected_++;
        if (config_.log_rejections) {
            std::cout << "[Mahalanobis] Rejected: D=" << mahalanobis_dist
                      << " threshold=" << std::sqrt(config_.chi_sq_threshold)
                      << " rate=" << rejection_rate() << std::endl;
        }
        return false;  // REJECTED: outlier
    }
    
    // Store innovation for adaptive threshold
    innovation_history_.push_back(mahalanobis_dist);
    if (innovation_history_.size() > (size_t)config_.history_window)
        innovation_history_.pop_front();
    
    return true;  // ACCEPTED
}

bool MahalanobisGater::validate_anatomy(const HandData& hand) {
    if (!hand.detected) return false;
    
    // First frame: initialize bone lengths
    if (!bone_lengths_.initialized) {
        bone_lengths_.initialize(hand);
        return true;
    }
    
    // Validate bone lengths
    bool bones_ok = bone_lengths_.is_valid(hand, config_.max_bone_length_change);
    
    // Auto-correct: update bone model on valid frames
    if (bones_ok && config_.auto_correct_bones) {
        bone_lengths_.initialize(hand);  // Sliding update
    }
    
    return bones_ok;
}

void MahalanobisGater::update_threshold(double rejection_rate) {
    // Adaptive: if too many rejections → loosen threshold
    // If too few → tighten threshold
    double target_rate = 0.01;  // 1% rejection target
    
    if (rejection_rate > target_rate * 2) {
        // Too many rejections: increase threshold
        config_.chi_sq_threshold *= 1.1;
    } else if (rejection_rate < target_rate * 0.5 && config_.chi_sq_threshold > 5.0) {
        // Too few rejections: decrease threshold
        config_.chi_sq_threshold *= 0.95;
    }
}

void MahalanobisGater::reset() {
    total_ = 0;
    rejected_ = 0;
    innovation_history_.clear();
    bone_lengths_ = BoneLengths{};
    config_.chi_sq_threshold = 14.16;
}

} // namespace hlp
