#include "PredictiveKalman.h"
#include <algorithm>
#include <cstring>
#include <iostream>

namespace hlp {

// ==================== PredictiveKalman ====================

PredictiveKalman::PredictiveKalman() {
    reset();
}

void PredictiveKalman::reset() {
    memset(state_, 0, sizeof(state_));
    p_.set_identity();
    tracking_state_ = TrackingState::TRACKING;
    lost_counter_ = 0;
    innovation_mag_ = 0;
    innovation_history_.clear();
    predicted_ = dvec3(0);
    predict_counter_ = 0;
    
    // Initialize Q (process noise)
    // Only position and velocity diagonals
    memset(q_.data, 0, sizeof(q_.data));
    q_.at(0,0) = config_.process_noise_pos;
    q_.at(1,1) = config_.process_noise_pos;
    q_.at(2,2) = config_.process_noise_pos;
    q_.at(3,3) = config_.process_noise_vel;
    q_.at(4,4) = config_.process_noise_vel;
    q_.at(5,5) = config_.process_noise_vel;
    
    // Initialize R (measurement noise)
    for (int i = 0; i < 3; i++)
        r_[i * 3 + i] = config_.measurement_noise;
}

void PredictiveKalman::reset(const dvec3& position) {
    reset();
    state_[0] = position.x;
    state_[1] = position.y;
    state_[2] = position.z;
    predicted_ = position;
}

void PredictiveKalman::mat66_mult(const double* a, const double* b, double* out) {
    memset(out, 0, 36 * sizeof(double));
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                out[i * 6 + j] += a[i * 6 + k] * b[k * 6 + j];
            }
        }
    }
}

void PredictiveKalman::mat66_add(double* a, const double* b) {
    for (int i = 0; i < 36; i++) a[i] += b[i];
}

double PredictiveKalman::determinant_33(const double* m) {
    return m[0] * (m[4] * m[8] - m[5] * m[7])
         - m[3] * (m[1] * m[8] - m[2] * m[7])
         + m[6] * (m[1] * m[5] - m[2] * m[4]);
}

void PredictiveKalman::predict(double dt) {
    // State transition matrix F (6x6)
    // [1 0 0 dt 0  0 ]
    // [0 1 0 0  dt 0 ]
    // [0 0 1 0  0  dt]
    // [0 0 0 1  0  0 ]
    // [0 0 0 0  1  0 ]
    // [0 0 0 0  0  1 ]
    
    // Position prediction: x = x + vx * dt
    state_[0] += state_[3] * dt;
    state_[1] += state_[4] * dt;
    state_[2] += state_[5] * dt;
    
    // Covariance prediction: P = F * P * F^T + Q
    double ft[36];
    memset(ft, 0, sizeof(ft));
    for (int i = 0; i < 6; i++) ft[i * 6 + i] = 1.0;
    ft[0*6+3] = dt;
    ft[1*6+4] = dt;
    ft[2*6+5] = dt;
    
    double fpf[36], fp[36];
    mat66_mult(p_.data, ft, fp);        // P * F^T
    mat66_mult(ft, fp, fpf);            // F * (P * F^T)
    memcpy(p_.data, fpf, sizeof(fpf));
    mat66_add(p_.data, q_.data);         // P = FPF^T + Q

    // Update process noise adaptively
    if (config_.adaptive_noise && tracking_state_ == TrackingState::TRACKING) {
        update_process_noise();
    }
}

void PredictiveKalman::correct(const dvec3& measurement) {
    // Innovation: y = z - H * x
    // H extracts position from state: [I_3x3 | 0_3x3]
    double y[3] = {
        measurement.x - state_[0],
        measurement.y - state_[1],
        measurement.z - state_[2]
    };
    
    innovation_mag_ = std::sqrt(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
    
    // Store innovation for adaptive noise
    if (config_.adaptive_noise) {
        innovation_history_.push_back(innovation_mag_);
        if (innovation_history_.size() > (size_t)config_.adaptation_window)
            innovation_history_.pop_front();
    }
    
    // S = H * P * H^T + R
    // H = [I_3x3 0_3x3], so H*P*H^T = top-left 3x3 of P
    double s[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            s[i*3+j] = p_.at(i, j) + r_[i*3+j];
    
    // Kalman gain: K = P * H^T * S^{-1}
    double det_s = determinant_33(s);
    if (std::abs(det_s) < 1e-20) {
        // Fallback: skip correction if singular
        return;
    }
    
    double inv_s[9];
    double inv_det = 1.0 / det_s;
    inv_s[0] = (s[4]*s[8] - s[5]*s[7]) * inv_det;
    inv_s[1] = (s[2]*s[7] - s[1]*s[8]) * inv_det;
    inv_s[2] = (s[1]*s[5] - s[2]*s[4]) * inv_det;
    inv_s[3] = (s[5]*s[6] - s[3]*s[8]) * inv_det;
    inv_s[4] = (s[0]*s[8] - s[2]*s[6]) * inv_det;
    inv_s[5] = (s[2]*s[3] - s[0]*s[5]) * inv_det;
    inv_s[6] = (s[3]*s[7] - s[4]*s[6]) * inv_det;
    inv_s[7] = (s[1]*s[6] - s[0]*s[7]) * inv_det;
    inv_s[8] = (s[0]*s[4] - s[1]*s[3]) * inv_det;
    
    // K = P * H^T * S^{-1}
    // (P * H^T) = first 3 columns of P
    double kh[3];  // K * y
    for (int i = 0; i < 6; i++) {
        double k_gain = 0;
        for (int j = 0; j < 3; j++) {
            double pk = 0;
            for (int k = 0; k < 3; k++)
                pk += p_.at(i, k) * inv_s[k*3 + j];
            k_gain += pk * y[j];
        }
        state_[i] += k_gain;
    }
    
    // Covariance update: P = (I - K*H) * P
    // Simplified: just reduce P
    for (int i = 0; i < 6; i++) {
        double hp = 0;
        for (int j = 0; j < 3; j++)
            hp += p_.at(i, j) * inv_s[j*3 + i]; // approximation
        for (int j = 0; j < 6; j++)
            p_.at(i, j) *= (1.0 - hp * 0.5);  // K*H approximation
    }
}

void PredictiveKalman::update_process_noise() {
    if (innovation_history_.size() < 5) return;
    
    // Compute variance of recent innovations
    double mean = 0;
    for (auto v : innovation_history_) mean += v;
    mean /= innovation_history_.size();
    
    double variance = 0;
    for (auto v : innovation_history_) variance += (v - mean) * (v - mean);
    variance /= innovation_history_.size();
    
    // If variance is high → increase process noise (more smoothing)
    // If variance is low → decrease process noise (more responsiveness)
    double scale = std::clamp(variance * 10.0, 
                               config_.noise_scale_min,
                               config_.noise_scale_max);
    
    q_.at(0,0) = config_.process_noise_pos * scale;
    q_.at(1,1) = config_.process_noise_pos * scale;
    q_.at(2,2) = config_.process_noise_pos * scale;
    q_.at(3,3) = config_.process_noise_vel * scale;
    q_.at(4,4) = config_.process_noise_vel * scale;
    q_.at(5,5) = config_.process_noise_vel * scale;
}

void PredictiveKalman::compute_predicted_position() {
    // Predict N frames ahead using current velocity
    double dt_pred = config_.dt * config_.predict_frames;
    
    predicted_.x = state_[0] + state_[3] * dt_pred;
    predicted_.y = state_[1] + state_[4] * dt_pred;
    predicted_.z = state_[2] + state_[5] * dt_pred;
}

dvec3 PredictiveKalman::update(const dvec3& measurement, double dt, bool tracking_valid) {
    config_.dt = dt;
    
    if (!tracking_valid) {
        lost_counter_++;
        
        if (lost_counter_ >= config_.max_lost_frames) {
            tracking_state_ = TrackingState::LOST;
            return predicted_;  // Return last known prediction
        }
        
        tracking_state_ = TrackingState::PREDICTING;
        
        // During predict: just propagate forward with damping
        state_[3] *= 0.95;  // Damp velocity
        state_[4] *= 0.95;
        state_[5] *= 0.95;
        
        predict(dt);
        compute_predicted_position();
        
        // Check if prediction has drifted too far
        if (innovation_mag_ > config_.drift_threshold && lost_counter_ > 5) {
            tracking_state_ = TrackingState::LOST;
        }
        
        return predicted_;
    }
    
    // Valid tracking: normal KF update
    if (tracking_state_ == TrackingState::LOST) {
        // Re-initialize
        reset(measurement);
        tracking_state_ = TrackingState::TRACKING;
        lost_counter_ = 0;
        return measurement;
    }
    
    // Predict then correct
    predict(dt);
    correct(measurement);
    compute_predicted_position();
    
    tracking_state_ = TrackingState::TRACKING;
    lost_counter_ = 0;
    
    // Return predicted position (ahead by predict_frames)
    return predicted_;
}

// ==================== TrackingStateMachine ====================

void TrackingStateMachine::reset() {
    state_ = INIT;
    frame_counter_ = 0;
    low_confidence_counter_ = 0;
    lost_counter_ = 0;
    recent_confidences_.clear();
    velocity_ = dvec3(0);
}

TrackingStateMachine::State TrackingStateMachine::update(const FrameInfo& frame) {
    frame_counter_++;
    
    if (!frame.valid) {
        lost_counter_++;
        
        switch (state_) {
            case INIT:
            case CALIBRATING:
                state_ = INIT;
                break;
            case TRACKING:
            case DEGRADED:
                state_ = (lost_counter_ > predict_timeout_) ? LOST : PREDICTING;
                break;
            case PREDICTING:
                if (lost_counter_ > predict_timeout_) state_ = LOST;
                break;
            default: break;
        }
        
        return state_;
    }
    
    // Valid measurement
    lost_counter_ = 0;
    
    // Store velocity
    if (frame_counter_ > 1) {
        velocity_ = (frame.position - last_known_position_) * 60.0;  // ~dt
    }
    last_known_position_ = frame.position;
    
    // Track confidence
    recent_confidences_.push_back(frame.confidence);
    if (recent_confidences_.size() > STAT_WINDOW)
        recent_confidences_.pop_front();
    
    switch (state_) {
        case INIT:
            if (recent_confidences_.size() >= 5) {
                state_ = CALIBRATING;
            }
            break;
            
        case CALIBRATING:
            if (recent_confidences_.size() >= STAT_WINDOW) {
                state_ = TRACKING;
            }
            break;
            
        case TRACKING:
            if (frame.confidence < degraded_conf_) {
                low_confidence_counter_++;
                if (low_confidence_counter_ > 5) {
                    state_ = DEGRADED;
                }
            } else {
                low_confidence_counter_ = 0;
            }
            break;
            
        case DEGRADED:
            if (frame.confidence >= degraded_conf_) {
                low_confidence_counter_ = 0;
                state_ = TRACKING;
            }
            break;
            
        case PREDICTING:
        case LOST:
            // Re-acquired
            state_ = TRACKING;
            low_confidence_counter_ = 0;
            break;
    }
    
    return state_;
}

} // namespace hlp
