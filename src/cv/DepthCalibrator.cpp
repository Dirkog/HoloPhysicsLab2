#include "DepthCalibrator.h"
#include <algorithm>
#include <iostream>

namespace hlp {

// ==================== HysteresisController ====================

double HysteresisController::measure_hand_size(const HandData& hand) const {
    // Use distance from wrist to middle finger tip as hand size reference
    float dx = hand[HAND_MIDDLE_TIP].x - hand[HAND_WRIST].x;
    float dy = hand[HAND_MIDDLE_TIP].y - hand[HAND_WRIST].y;
    return std::sqrt(dx * dx + dy * dy);
}

bool HysteresisController::is_pinching(const HandData& hand, double hand_size) const {
    if (hand_size < 1.0) return false;
    
    auto& thumb = hand[HAND_THUMB_TIP];
    auto& index = hand[HAND_INDEX_TIP];
    float dx = thumb.x - index.x;
    float dy = thumb.y - index.y;
    float dz = (thumb.z - index.z) * 100;
    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    // Normalize by hand size
    float ratio = dist / hand_size;
    
    // Hysteresis: different thresholds for entering vs exiting
    if (stable_gesture_ == HandGesture::PINCH) {
        return ratio < config_.pinch_open_ratio;   // Stays pinched longer
    } else {
        return ratio < config_.pinch_close_ratio;   // Requires closer to enter
    }
}

bool HysteresisController::is_fist(const HandData& hand, double hand_size) const {
    if (hand_size < 1.0) return false;
    
    float total_dist = 0;
    for (int tip : {HAND_INDEX_TIP, HAND_MIDDLE_TIP, HAND_RING_TIP, HAND_PINKY_TIP}) {
        float dx = hand[tip].x - hand[HAND_WRIST].x;
        float dy = hand[tip].y - hand[HAND_WRIST].y;
        total_dist += std::sqrt(dx * dx + dy * dy);
    }
    float avg_dist = total_dist / 4.0f;
    float ratio = avg_dist / hand_size;
    
    if (stable_gesture_ == HandGesture::FIST) {
        return ratio < config_.fist_open_ratio;
    } else {
        return ratio < config_.fist_close_ratio;
    }
}

int HysteresisController::count_extended_fingers(const HandData& hand) const {
    int count = 0;
    // Thumb: extended if tip is far from palm
    float thumb_dist = std::sqrt(
        std::pow(hand[HAND_THUMB_TIP].x - hand[HAND_WRIST].x, 2) +
        std::pow(hand[HAND_THUMB_TIP].y - hand[HAND_WRIST].y, 2));
    if (thumb_dist > 30) count++;
    
    // Other fingers: tip "higher" than PIP in image space
    auto check = [&](int tip, int pip) {
        if (hand[tip].y < hand[pip].y - 0.02f) count++;
    };
    check(HAND_INDEX_TIP, HAND_INDEX_PIP);
    check(HAND_MIDDLE_TIP, HAND_MIDDLE_PIP);
    check(HAND_RING_TIP, HAND_RING_PIP);
    check(HAND_PINKY_TIP, HAND_PINKY_PIP);
    
    return count;
}

HandGesture HysteresisController::apply(const HandData& hand_data, 
                                          HandGesture raw_gesture,
                                          double hand_size, double dt) {
    (void)dt;
    
    // 1. Confidence gate
    if (hand_data.confidence < config_.min_confidence) {
        // Freeze: keep last stable gesture, don't change
        return stable_gesture_;
    }
    
    // 2. Determine what the actual gesture should be based on hysteresis
    HandGesture desired;
    bool pinch = is_pinching(hand_data, hand_size);
    bool fist = is_fist(hand_data, hand_size);
    int fingers = count_extended_fingers(hand_data);
    
    if (pinch) {
        desired = HandGesture::PINCH;
    } else if (fist) {
        desired = HandGesture::FIST;
    } else if (fingers >= 4) {
        desired = HandGesture::OPEN_PALM;
    } else if (fingers == 1) {
        desired = HandGesture::POINTING;
    } else {
        desired = HandGesture::UNKNOWN;
    }
    
    // 3. Debounce: require N consecutive frames of same gesture
    if (desired == last_raw_) {
        if (desired != stable_gesture_) {
            raw_counter_++;
            debounce_progress_ = (float)raw_counter_ / config_.debounce_frames;
            
            if (raw_counter_ >= config_.debounce_frames) {
                // Transition: was old gesture → now new gesture
                if (stable_gesture_ == HandGesture::PINCH && desired != HandGesture::PINCH) {
                    stable_gesture_ = HandGesture::PINCH_OPEN;
                } else if (stable_gesture_ != HandGesture::PINCH && desired == HandGesture::PINCH) {
                    stable_gesture_ = HandGesture::PINCH_CLOSE;
                } else {
                    stable_gesture_ = desired;
                }
                raw_counter_ = 0;
                debounce_progress_ = 1.0f;
            }
        } else {
            // Already in this state
            raw_counter_ = 0;
            debounce_progress_ = 1.0f;
        }
    } else {
        // Gesture changed before debounce completed
        raw_counter_ = 0;
        debounce_progress_ = 0;
        
        // For release: immediate if we were grabbing and hand is now open
        if (stable_gesture_ == HandGesture::PINCH && desired != HandGesture::PINCH) {
            release_counter_++;
            if (release_counter_ >= config_.release_debounce) {
                stable_gesture_ = HandGesture::PINCH_OPEN;
                release_counter_ = 0;
            }
            return stable_gesture_;  // Hold during release debounce
        }
        release_counter_ = 0;
    }
    
    // 4. Transition from PINCH to PINCH_CLOSE → next frame becomes PINCH
    if (stable_gesture_ == HandGesture::PINCH_CLOSE && last_raw_ != HandGesture::PINCH) {
        stable_gesture_ = HandGesture::PINCH;
    }
    
    last_raw_ = desired;
    return stable_gesture_;
}

void HysteresisController::reset() {
    stable_gesture_ = HandGesture::UNKNOWN;
    last_raw_ = HandGesture::UNKNOWN;
    raw_counter_ = 0;
    release_counter_ = 0;
    debounce_progress_ = 0;
}

// ==================== ZStabilizer ====================

double ZStabilizer::update(double palm_width_px, const CalibrationData& calib, double dt) {
    if (!initialized_) {
        depth_ = calib.estimate_depth(palm_width_px);
        prev_raw_depth_ = depth_;
        
        // Initialize median filter
        depth_history_.clear();
        for (int i = 0; i < MEDIAN_WINDOW; i++)
            depth_history_.push_back(depth_);
        
        initialized_ = true;
        return depth_;
    }
    
    double raw_depth = calib.estimate_depth(palm_width_px);
    
    // 1. Median filter (removes outliers)
    depth_history_.push_back(raw_depth);
    if (depth_history_.size() > MEDIAN_WINDOW)
        depth_history_.pop_front();
    
    std::deque<double> sorted = depth_history_;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];
    
    // 2. Clamp velocity (avoids Z-jumps from tracking errors)
    double max_delta = max_delta_per_sec_ * dt;
    double clamped = std::clamp(median, 
                                 prev_raw_depth_ - max_delta,
                                 prev_raw_depth_ + max_delta);
    prev_raw_depth_ = clamped;
    
    // 3. Strong EMA smoothing (much stronger than XY because Z is inherently noisy)
    depth_ = depth_ * (1.0 - smoothing_) + clamped * smoothing_;
    
    // 4. Velocity estimation for prediction
    velocity_ema_ = velocity_ema_ * 0.8 + ((depth_ - prev_raw_depth_) / dt) * 0.2;
    
    return depth_;
}

void ZStabilizer::reset() {
    initialized_ = false;
    depth_ = 0.5;
    prev_raw_depth_ = 0.5;
    depth_history_.clear();
    velocity_ema_ = 0;
}

} // namespace hlp
