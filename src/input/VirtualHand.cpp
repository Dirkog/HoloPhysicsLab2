#include "VirtualHand.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace hlp {

const char* gesture_name(HandGesture g) {
    switch (g) {
        case HandGesture::OPEN_PALM:    return "OPEN_PALM";
        case HandGesture::PINCH:        return "PINCH";
        case HandGesture::FIST:         return "FIST";
        case HandGesture::POINTING:     return "POINTING";
        case HandGesture::PINCH_CLOSE:  return "PINCH_CLOSE";
        case HandGesture::PINCH_OPEN:   return "PINCH_OPEN";
        case HandGesture::SWIPE_LEFT:   return "SWIPE_LEFT";
        case HandGesture::SWIPE_RIGHT:  return "SWIPE_RIGHT";
        case HandGesture::SWIPE_UP:     return "SWIPE_UP";
        case HandGesture::SWIPE_DOWN:   return "SWIPE_DOWN";
        case HandGesture::RELAX:        return "RELAX";
        default:                        return "UNKNOWN";
    }
}

// ==================== KalmanFilter1D ====================

KalmanFilter1D::KalmanFilter1D(double q, double r) : q_(q), r_(r) {}

double KalmanFilter1D::update(double measurement) {
    // Predict
    p_ = p_ + q_;
    
    // Update
    k_ = p_ / (p_ + r_);
    prev_x_ = x_;
    x_ = x_ + k_ * (measurement - x_);
    p_ = (1 - k_) * p_;
    
    return x_;
}

void KalmanFilter1D::reset(double value) {
    x_ = value;
    prev_x_ = value;
    p_ = 1;
}

// ==================== KalmanFilter3D ====================

KalmanFilter3D::KalmanFilter3D(double process_noise, double measurement_noise)
    : kfx_(process_noise, measurement_noise)
    , kfy_(process_noise, measurement_noise)
    , kfz_(process_noise, measurement_noise) {}

dvec3 KalmanFilter3D::update(const dvec3& measurement) {
    dvec3 prev = pos_;
    
    pos_.x = kfx_.update(measurement.x);
    pos_.y = kfy_.update(measurement.y);
    pos_.z = kfz_.update(measurement.z);
    
    vel_ = pos_ - prev;
    return pos_;
}

void KalmanFilter3D::reset(const dvec3& value) {
    kfx_.reset(value.x);
    kfy_.reset(value.y);
    kfz_.reset(value.z);
    pos_ = value;
    vel_ = dvec3(0);
}

// ==================== GestureRecognizer ====================

GestureRecognizer::GestureRecognizer() = default;

float GestureRecognizer::calc_pinch_distance(const HandData& hand) {
    auto& thumb = hand[HAND_THUMB_TIP];
    auto& index = hand[HAND_INDEX_TIP];
    float dx = thumb.x - index.x;
    float dy = thumb.y - index.y;
    float dz = (thumb.z - index.z) * 100;  // MediaPipe z-scale
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int GestureRecognizer::count_extended_fingers(const HandData& hand) {
    int count = 0;
    // Thumb: check distance from palm
    float thumb_dist = hand.finger_distance(HAND_THUMB_TIP, HAND_THUMB_IP);
    float thumb_ref = hand.finger_distance(HAND_INDEX_MCP, HAND_INDEX_PIP);
    if (thumb_dist > thumb_ref * 0.5) count++;
    
    // Other fingers: tip.y < pip.y (image space)
    if (hand.is_finger_extended(HAND_INDEX_TIP, HAND_INDEX_PIP)) count++;
    if (hand.is_finger_extended(HAND_MIDDLE_TIP, HAND_MIDDLE_PIP)) count++;
    if (hand.is_finger_extended(HAND_RING_TIP, HAND_RING_PIP)) count++;
    if (hand.is_finger_extended(HAND_PINKY_TIP, HAND_PINKY_PIP)) count++;
    
    return count;
}

bool GestureRecognizer::is_fist(const HandData& hand) {
    float total_dist = 0;
    for (int tip : {HAND_INDEX_TIP, HAND_MIDDLE_TIP, HAND_RING_TIP, HAND_PINKY_TIP}) {
        total_dist += hand.finger_distance(tip, HAND_WRIST);
    }
    return total_dist < 1.5f;  // Fingertips close to palm
}

bool GestureRecognizer::is_pointing(const HandData& hand) {
    int extended = count_extended_fingers(hand);
    return extended == 1 && hand.is_finger_extended(HAND_INDEX_TIP, HAND_INDEX_PIP);
}

float GestureRecognizer::hand_openness(const HandData& hand) {
    // [0,1] where 0 = fist, 1 = open palm
    float total = 0;
    for (int tip : {HAND_INDEX_TIP, HAND_MIDDLE_TIP, HAND_RING_TIP, HAND_PINKY_TIP}) {
        for (int pip : {HAND_INDEX_PIP, HAND_MIDDLE_PIP, HAND_RING_PIP, HAND_PINKY_PIP}) {
            // Not elegant but works
        }
    }
    // Simpler: average distance from fingertips to palm
    float avg_dist = 0;
    for (int tip : {HAND_INDEX_TIP, HAND_MIDDLE_TIP, HAND_RING_TIP, HAND_PINKY_TIP}) {
        avg_dist += hand.finger_distance(tip, HAND_WRIST);
    }
    avg_dist /= 4.0f;
    return std::clamp(avg_dist / 0.5f, 0.0f, 1.0f);
}

HandGesture GestureRecognizer::classify(const HandData& hand, float& pinch_strength) {
    if (!hand.detected || hand.confidence < 0.5f) {
        return HandGesture::UNKNOWN;
    }
    
    pinch_strength = std::clamp(1.0f - calc_pinch_distance(hand) * 3.0f, 0.0f, 1.0f);
    int fingers = count_extended_fingers(hand);
    
    // Priority: PINCH > FIST > POINTING > OPEN_PALM
    
    // Pinch
    if (pinch_strength > 0.6f && fingers <= 3) {
        // Transition detection
        HandGesture result = HandGesture::PINCH;
        if (prev_gesture_ != HandGesture::PINCH && 
            prev_gesture_ != HandGesture::PINCH_CLOSE) {
            result = HandGesture::PINCH_CLOSE;
        }
        prev_gesture_ = HandGesture::PINCH;
        return result;
    }
    
    // Fist
    if (fingers <= 1 && is_fist(hand)) {
        prev_gesture_ = HandGesture::FIST;
        return HandGesture::FIST;
    }
    
    // Pointing (only index extended)
    if (fingers == 1 && is_pointing(hand)) {
        prev_gesture_ = HandGesture::POINTING;
        return HandGesture::POINTING;
    }
    
    // Open palm
    if (fingers >= 4) {
        prev_gesture_ = HandGesture::OPEN_PALM;
        return HandGesture::OPEN_PALM;
    }
    
    // Transition: was PINCH → now not PINCH
    if (prev_gesture_ == HandGesture::PINCH) {
        prev_gesture_ = HandGesture::UNKNOWN;
        return HandGesture::PINCH_OPEN;
    }
    
    prev_gesture_ = HandGesture::UNKNOWN;
    return HandGesture::UNKNOWN;
}

HandGesture GestureRecognizer::detect_swipe(const dvec3& velocity, double threshold) {
    double vx = velocity.x, vy = velocity.y;
    double abs_vx = std::abs(vx), abs_vy = std::abs(vy);
    
    if (abs_vx < threshold && abs_vy < threshold)
        return HandGesture::UNKNOWN;
    
    if (abs_vx > abs_vy) {
        return vx > 0 ? HandGesture::SWIPE_RIGHT : HandGesture::SWIPE_LEFT;
    } else {
        return vy > 0 ? HandGesture::SWIPE_DOWN : HandGesture::SWIPE_UP;
    }
}

// ==================== VirtualHand ====================

void VirtualHand::update(const HandData& raw_hand, double dt) {
    was_active_ = active_;
    prev_gesture_ = gesture_;
    
    if (!raw_hand.detected) {
        active_ = false;
        reset();
        return;
    }
    
    active_ = true;
    
    // 1. Преобразование 2D landmarks → 3D мировые координаты
    for (int i = 0; i < HAND_LANDMARK_COUNT; i++) {
        auto& wp = raw_hand.world_positions[i];
        world_[i] = dvec3(wp.x, wp.y, wp.z);
    }
    
    // 2. Ключевые точки
    wrist_ = world_[HAND_WRIST];
    palm_center_ = dvec3(0, 0, 0);
    for (int i : {HAND_THUMB_CMC, HAND_INDEX_MCP, HAND_MIDDLE_MCP,
                  HAND_RING_MCP, HAND_PINKY_MCP}) {
        palm_center_ += world_[i];
    }
    palm_center_ /= 5.0;
    index_tip_ = world_[HAND_INDEX_TIP];
    
    // 3. Калмановская фильтрация
    kf_.update(palm_center_);
    
    // 4. Распознавание жеста
    gesture_ = recognizer_.classify(raw_hand, pinch_strength_);
    openness_ = GestureRecognizer::hand_openness(raw_hand);
    
    // 5. Детектор свайпов
    if (gesture_ == HandGesture::UNKNOWN || gesture_ == HandGesture::OPEN_PALM) {
        dvec3 vel = kf_.velocity();
        if (glm::length(vel) > 0.5) {
            HandGesture swipe = recognizer_.detect_swipe(vel, 0.3);
            if (swipe != HandGesture::UNKNOWN) {
                gesture_ = swipe;
            }
        }
    }
    
    // 6. Сглаженная скорость
    if (has_prev_pos_) {
        dvec3 raw_vel = (palm_center_ - prev_pos_) / std::max(dt, 1e-6);
        smooth_vel_ = smooth_vel_ * 0.7 + raw_vel * 0.3;
    }
    prev_pos_ = palm_center_;
    has_prev_pos_ = true;
    
    // 7. История для сглаживания
    position_history_.push_back(palm_center_);
    if (position_history_.size() > HISTORY_SIZE)
        position_history_.pop_front();
    
    // 8. Обработка перехода PINCH_CLOSE → PINCH
    if (gesture_ == HandGesture::PINCH_CLOSE) {
        // В следующем кадре будет просто PINCH
        gesture_ = HandGesture::PINCH;
    }
    
    // Callback через GestureController
}

bool VirtualHand::just_became_active() const {
    return active_ && !was_active_;
}

bool VirtualHand::just_became_inactive() const {
    return !active_ && was_active_;
}

void VirtualHand::reset() {
    interaction_.reset();
    position_history_.clear();
    has_prev_pos_ = false;
    smooth_vel_ = dvec3(0);
    pinch_strength_ = 0;
}

// ==================== GestureController ====================

GestureController::GestureController() = default;

void GestureController::calibrate() {
    calibration_frames_ = 0;
    calibrated_ = false;
}

void GestureController::update(HandTracker& tracker, 
                                const CameraIntrinsics& intrinsics,
                                double dt) {
    if (!enabled_) return;
    
    for (int i = 0; i < tracker.hand_count(); i++) {
        auto& raw = tracker.hand(i);
        
        // 2D → 3D unprojection
        tracker.unproject_to_3d(raw, intrinsics);
        
        // Update virtual hand
        VirtualHand& vhand = (raw.hand_id == 0) ? right_hand_ : left_hand_;
        vhand.is_right_hand = (raw.hand_id == 0);
        vhand.update(raw, dt);
        
        // Gesture change callback
        if (on_gesture_change && vhand.gesture() != vhand.prev_gesture()) {
            on_gesture_change(vhand, vhand.prev_gesture(), vhand.gesture());
        }
    }
    
    // Калибровка
    if (!calibrated_) {
        calibration_frames_++;
        if (calibration_frames_ >= CALIBRATION_FRAMES) {
            calibrated_ = true;
            std::cout << "[GestureController] Calibrated after " 
                      << calibration_frames_ << " frames" << std::endl;
        }
    }
}

} // namespace hlp
