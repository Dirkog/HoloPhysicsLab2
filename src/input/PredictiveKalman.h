#pragma once

// ============================================================
// HoloPhysics Lab 2 — Predictive Kalman Filter
// 
// Решает проблемы:
//   1. Латентность CV-пайплайна (30-50ms)
//   2. Предсказание позиции на 1-2 кадра вперёд
//   3. Graceful degradation при потере трекинга
//   4. Адаптивный Q/R по вариации измерений
// ============================================================

#include "../core/Types.h"
#include <deque>
#include <cmath>
#include <optional>

namespace hlp {

// ==================== State Vector ====================
// State: [x, y, z, vx, vy, vz]
// Measurement: [x, y, z]

struct KalmanConfig {
    double process_noise_pos = 0.005;   // Q_pos: uncertainty in position
    double process_noise_vel = 0.05;    // Q_vel: uncertainty in velocity  
    double measurement_noise = 0.05;    // R: measurement uncertainty
    double initial_error = 1.0;         // P0: initial error covariance
    
    // Adaptive tuning
    bool adaptive_noise = true;
    int adaptation_window = 10;          // Frames for noise estimation
    double noise_scale_min = 0.001;
    double noise_scale_max = 1.0;
    
    // Prediction
    int predict_frames = 1;             // How many frames to predict ahead
    double dt = 0.016;                  // Timestep (16ms default)
    
    // Tracking loss
    int max_lost_frames = 30;           // After this many lost frames, reset
    double drift_threshold = 0.5;       // Max prediction drift before declaring loss
};

// 6x6 covariance matrix stored as array
struct CovMatrix6 {
    double data[36] = {0};
    
    void set_identity() {
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                data[i * 6 + j] = (i == j) ? 1.0 : 0.0;
    }
    
    double& at(int i, int j) { return data[i * 6 + j]; }
    const double& at(int i, int j) const { return data[i * 6 + j]; }
};

class PredictiveKalman {
public:
    PredictiveKalman();
    ~PredictiveKalman() = default;
    
    // === Core update ===
    // Returns predicted position (smoothed + predicted ahead by predict_frames)
    dvec3 update(const dvec3& measurement, double dt, bool tracking_valid);
    
    // === State queries ===
    dvec3 position() const { return dvec3(state_[0], state_[1], state_[2]); }
    dvec3 velocity() const { return dvec3(state_[3], state_[4], state_[5]); }
    dvec3 predicted_position() const { return predicted_; }
    
    // === Tracking loss ===
    enum class TrackingState {
        TRACKING,        // Normal operation
        PREDICTING,      // Lost but still predicting
        LOST             // Gave up, needs re-initialization
    };
    
    TrackingState tracking_state() const { return tracking_state_; }
    int frames_since_valid() const { return lost_counter_; }
    bool is_stable() const { return tracking_state_ == TrackingState::TRACKING; }
    
    // === Configuration ===
    void set_config(const KalmanConfig& cfg) { config_ = cfg; }
    const KalmanConfig& config() const { return config_; }
    KalmanConfig& config() { return config_; }
    
    // === Reset ===
    void reset();
    void reset(const dvec3& position);
    
    // === Diagnostics ===
    double innovation() const { return innovation_mag_; }
    double position_uncertainty() const { return sqrt(p_[0*6+0] + p_[1*6+1] + p_[2*6+2]); }
    const CovMatrix6& covariance() const { return p_; }

private:
    // State
    double state_[6] = {0};  // [x, y, z, vx, vy, vz]
    CovMatrix6 p_;            // Error covariance (6x6)
    
    // Process noise (6x6)
    CovMatrix6 q_;
    
    // Measurement noise (3x3, embedded in 6x6)
    double r_[9] = {0};
    
    // Prediction
    dvec3 predicted_{0};
    int predict_counter_ = 0;
    
    // Tracking loss
    KalmanConfig config_;
    TrackingState tracking_state_ = TrackingState::TRACKING;
    int lost_counter_ = 0;
    
    // Innovation (measurement residual)
    double innovation_mag_ = 0;
    
    // History for adaptive noise
    std::deque<double> innovation_history_;
    
    // Forward prediction
    void predict(double dt);
    void correct(const dvec3& measurement);
    void update_process_noise();
    void compute_predicted_position();
    
    // Helper
    static void mat66_mult(const double* a, const double* b, double* out);
    static void mat66_add(double* a, const double* b);
    static double determinant_33(const double* m);
};

// ==================== Tracking State Machine ====================

class TrackingStateMachine {
public:
    TrackingStateMachine() = default;
    
    enum State {
        INIT,           // First frame, collecting initial data
        CALIBRATING,    // Gathering statistics (variance, typical velocity)
        TRACKING,       // Normal tracking
        DEGRADED,       // Low confidence but still tracking
        PREDICTING,     // No valid measurement, predicting forward
        LOST,           // Permanently lost, needs re-init
    };
    
    struct FrameInfo {
        dvec3 position;
        double confidence;
        bool valid;
        double timestamp;
    };
    
    State update(const FrameInfo& frame);
    State current_state() const { return state_; }
    
    // Graceful degradation settings
    void set_degraded_threshold(float c) { degraded_conf_ = c; }
    void set_predict_timeout(int frames) { predict_timeout_ = frames; }
    void set_max_drift(double d) { max_drift_ = d; }
    
    // Get recommended behavior
    bool should_predict() const { return state_ == PREDICTING || state_ == DEGRADED; }
    bool should_free_last() const { return state_ == DEGRADED; }
    bool should_reset() const { return state_ == LOST; }
    
    // Reset
    void reset();
    
    // Velocity for extrapolation during predict
    dvec3 current_velocity() const { return velocity_; }
    
private:
    State state_ = INIT;
    int frame_counter_ = 0;
    int low_confidence_counter_ = 0;
    int lost_counter_ = 0;
    
    float degraded_conf_ = 0.4f;
    int predict_timeout_ = 30;
    double max_drift_ = 0.5;
    
    dvec3 last_known_position_{0};
    dvec3 velocity_{0};
    double last_known_time_ = 0;
    
    // Statistics
    std::deque<double> recent_confidences_;
    static constexpr int STAT_WINDOW = 15;
};

} // namespace hlp
