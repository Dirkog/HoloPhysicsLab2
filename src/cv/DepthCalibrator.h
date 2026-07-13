#pragma once

// ============================================================
// HoloPhysics Lab 2 — Depth Calibration & Gesture Hysteresis
// 
// Решает проблемы:
//   1. Монокулярная оценка глубины по размеру ладони
//   2. Индивидуальная анатомия пользователя
//   3. Дребезг жестов (flickering)
//   4. Одноразовая калибровка с сохранением
// ============================================================

#include "../cv/HandTracker.h"
#include <deque>
#include <fstream>
#include <string>
#include <cmath>

namespace hlp {

// ==================== Depth Calibration ====================

struct CalibrationData {
    // Reference palm width in pixels at known distance
    double ref_palm_width_px = 120.0;  // Typical at 50cm with 640×480
    double ref_distance_m = 0.5;        // Reference distance (meters)
    
    // Camera intrinsics (estimated or from calibration)
    double focal_length_px = 615.0;
    
    // User's actual palm size in meters (after calibration)
    double user_palm_size_m = 0.085;    // Average adult palm width
    
    // Calibration state
    bool calibrated = false;
    int calibration_samples = 0;
    static constexpr int MIN_SAMPLES = 15;
    double sum_palm_width_px = 0;
    
    // Save/load
    std::string filename = "holo_hand_calibration.json";
    
    void reset() {
        calibrated = false;
        calibration_samples = 0;
        sum_palm_width_px = 0;
    }
    
    // Add measurement sample
    bool add_sample(double palm_width_px) {
        if (palm_width_px < 20 || palm_width_px > 500) return false;  // Sanity check
        sum_palm_width_px += palm_width_px;
        calibration_samples++;
        
        if (calibration_samples >= MIN_SAMPLES) {
            double avg_palm_px = sum_palm_width_px / calibration_samples;
            // user_palm_size_m = ref_distance_m * avg_palm_px / focal_length_px
            // But we want: if user's palm at 50cm shows X pixels, 
            // then user_palm_size_m = (X * ref_distance_m) / focal_length_px
            user_palm_size_m = (avg_palm_px * ref_distance_m) / focal_length_px;
            calibrated = true;
            return true;
        }
        return false;
    }
    
    // Depth from palm width: Z = (user_palm_size_m * focal_length_px) / palm_width_px
    double estimate_depth(double palm_width_px) const {
        if (palm_width_px < 1.0) return ref_distance_m;
        double depth = (user_palm_size_m * focal_length_px) / palm_width_px;
        return std::clamp(depth, 0.1, 2.0);
    }
    
    bool save_to_file() const {
        std::ofstream f(filename);
        if (!f) return false;
        f << "{\n";
        f << "  \"user_palm_size_m\": " << user_palm_size_m << ",\n";
        f << "  \"ref_distance_m\": " << ref_distance_m << ",\n";
        f << "  \"focal_length_px\": " << focal_length_px << ",\n";
        f << "  \"calibrated\": " << (calibrated ? "true" : "false") << "\n";
        f << "}\n";
        return true;
    }
    
    bool load_from_file() {
        std::ifstream f(filename);
        if (!f) return false;
        std::string line;
        while (std::getline(f, line)) {
            if (line.find("user_palm_size_m") != std::string::npos) {
                auto colon = line.find(':');
                if (colon != std::string::npos)
                    user_palm_size_m = std::stod(line.substr(colon + 1));
            }
            if (line.find("calibrated") != std::string::npos) {
                calibrated = line.find("true") != std::string::npos;
            }
        }
        return calibrated;
    }
};

// ==================== Gesture Hysteresis ====================

struct HysteresisConfig {
    // Pinch thresholds (normalized by hand size)
    double pinch_close_ratio = 0.15;   // Pinch activates when thumb-index < 15% of hand size
    double pinch_open_ratio = 0.25;    // Pinch deactivates when > 25% of hand size
    
    // Fist thresholds
    double fist_close_ratio = 0.30;     // Fist when fingertips-palm distance < 30% of hand size
    double fist_open_ratio = 0.40;      // Releases when > 40%
    
    // Debounce
    int debounce_frames = 3;            // Frames required for gesture to stabilize
    int release_debounce = 2;           // Frames required for release
    
    // Confidence gate
    float min_confidence = 0.5f;        // Below this: freeze last state
};

class HysteresisController {
public:
    HysteresisController() = default;
    
    void set_config(const HysteresisConfig& cfg) { config_ = cfg; }
    const HysteresisConfig& config() const { return config_; }
    
    // Apply hysteresis to raw gesture
    HandGesture apply(const HandData& hand_data, HandGesture raw_gesture, 
                      double hand_size, double dt);
    
    // Get debounce progress [0,1] for UI feedback
    float debounce_progress() const { return debounce_progress_; }
    
    // Reset
    void reset();
    
private:
    HysteresisConfig config_;
    
    // State machine
    HandGesture stable_gesture_ = HandGesture::UNKNOWN;
    HandGesture last_raw_ = HandGesture::UNKNOWN;
    int raw_counter_ = 0;
    int release_counter_ = 0;
    float debounce_progress_ = 0;
    
    // Hysteresis checks
    bool is_pinching(const HandData& hand, double hand_size) const;
    bool is_fist(const HandData& hand, double hand_size) const;
    int count_extended_fingers(const HandData& hand) const;
    double measure_hand_size(const HandData& hand) const;
};

// ==================== Z-Stabilizer ====================

class ZStabilizer {
public:
    ZStabilizer() = default;
    
    // Update with palm width, return stabilized Z
    double update(double palm_width_px, const CalibrationData& calib, double dt);
    
    // Get current Z
    double depth() const { return depth_; }
    
    // Reset
    void reset();
    
    // Configuration
    void set_smoothing(double smooth) { smoothing_ = smooth; }
    void set_max_delta(double max) { max_delta_per_sec_ = max; }
    
private:
    double depth_ = 0.5;
    double prev_raw_depth_ = 0.5;
    double smoothing_ = 0.15;       // Much stronger smoothing for Z than XY
    double max_delta_per_sec_ = 2.0; // Max depth change per second (avoids jumps)
    double velocity_ema_ = 0;
    bool initialized_ = false;
    
    // Median filter
    std::deque<double> depth_history_;
    static constexpr int MEDIAN_WINDOW = 7;
};

} // namespace hlp
