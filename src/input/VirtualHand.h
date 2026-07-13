#pragma once

// ============================================================
// HoloPhysics Lab 2 — Virtual Hand (Input Layer)
// Сглаживание, распознавание жестов, трекинг состояний
// Мост между CV (HandTracker) и Физикой (Engine)
// ============================================================

#include "../cv/HandTracker.h"
#include "../core/Types.h"
#include <deque>
#include <functional>
#include <glm/glm.hpp>

namespace hlp {

// ==================== Gesture Recognition ====================

enum class HandGesture : uint8_t {
    UNKNOWN = 0,
    OPEN_PALM,       // Все пальцы раскрыты
    PINCH,           // Большой + указательный вместе
    FIST,            // Кулак
    POINTING,        // Только указательный
    PINCH_CLOSE,     // Переход: было не PINCH → стало PINCH
    PINCH_OPEN,      // Переход: было PINCH → стало не PINCH
    SWIPE_LEFT,      // Быстрое движение влево
    SWIPE_RIGHT,     // Быстрое движение вправо
    SWIPE_UP,        // Вверх
    SWIPE_DOWN,      // Вниз
    RELAX,           // Расслабленная кисть (нейтральное)
};

const char* gesture_name(HandGesture g);

// ==================== Virtual Hand State ====================

struct HandInteractionState {
    // Текущий захват
    uint64_t grabbed_body_id = UINT64_MAX;
    bool is_grabbing = false;
    double grab_start_time = 0;
    dvec3 grab_offset{0}; // Смещение между пальцем и ЦМ тела
    
    // Dwell (для UI)
    double dwell_timer = 0;
    dvec3 dwell_position{0};
    bool dwell_activated = false;
    
    // Raycast
    uint64_t hovered_body_id = UINT64_MAX;
    dvec3 hover_point{0};
    
    // Throw
    dvec3 release_velocity{0};
    
    void reset() {
        grabbed_body_id = UINT64_MAX;
        is_grabbing = false;
        dwell_timer = 0;
        dwell_activated = false;
        hovered_body_id = UINT64_MAX;
    }
};

// ==================== Kalman Filter (1D) ====================

class KalmanFilter1D {
public:
    KalmanFilter1D(double process_noise = 0.01, double measurement_noise = 0.1);
    
    double update(double measurement);
    void reset(double value = 0);
    double value() const { return x_; }
    double velocity() const { return (x_ - prev_x_) / dt_; }
    
private:
    double x_ = 0;       // State estimate
    double p_ = 1;       // Error covariance
    double q_ = 0.01;    // Process noise
    double r_ = 0.1;     // Measurement noise
    double k_ = 0;       // Kalman gain
    double prev_x_ = 0;
    double dt_ = 0.016;
};

// ==================== 3D Kalman Filter ====================

class KalmanFilter3D {
public:
    KalmanFilter3D(double process_noise = 0.01, double measurement_noise = 0.1);
    
    dvec3 update(const dvec3& measurement);
    void reset(const dvec3& value = dvec3(0));
    dvec3 position() const { return pos_; }
    dvec3 velocity() const { return vel_; }
    
private:
    KalmanFilter1D kfx_, kfy_, kfz_;
    dvec3 pos_{0};
    dvec3 vel_{0};
};

// ==================== Gesture Recognizer ====================

class GestureRecognizer {
public:
    GestureRecognizer();
    
    // Анализ landmarks → жест
    HandGesture classify(const HandData& hand, float& pinch_strength);
    
    // Детектор свайпов
    HandGesture detect_swipe(const dvec3& velocity, double threshold = 0.3);
    
    // Помощники
    static float calc_pinch_distance(const HandData& hand);
    static int count_extended_fingers(const HandData& hand);
    static bool is_fist(const HandData& hand);
    static bool is_pointing(const HandData& hand);
    static float hand_openness(const HandData& hand);
    
private:
    HandGesture prev_gesture_ = HandGesture::UNKNOWN;
    dvec3 prev_palm_pos_{0};
    bool has_prev_ = false;
};

// ==================== Virtual Hand ====================

class VirtualHand {
public:
    VirtualHand() = default;
    
    // Главный метод обновления
    void update(const HandData& raw_hand, double dt);
    
    // Сглаженные 3D позиции (в мировых координатах)
    dvec3 index_tip() const { return index_tip_; }
    dvec3 palm_center() const { return palm_center_; }
    dvec3 wrist() const { return wrist_; }
    
    // Все 21 точка в мировых координатах
    const std::array<dvec3, HAND_LANDMARK_COUNT>& world_landmarks() const { return world_; }
    
    // Скорости
    dvec3 velocity() const { return kf_.velocity(); }
    dvec3 smoothed_velocity() const { return smooth_vel_; }
    
    // Жест
    HandGesture gesture() const { return gesture_; }
    HandGesture prev_gesture() const { return prev_gesture_; }
    float pinch_strength() const { return pinch_strength_; }
    float hand_openness() const { return openness_; }
    
    // Состояние взаимодействия
    HandInteractionState& interaction() { return interaction_; }
    const HandInteractionState& interaction() const { return interaction_; }
    
    // Активация
    bool is_active() const { return active_; }
    bool just_became_active() const;
    bool just_became_inactive() const;
    
    // Сброс
    void reset();
    
    // Принадлежность
    bool is_right_hand = true;
    
private:
    // Позиции
    dvec3 index_tip_{0};
    dvec3 palm_center_{0};
    dvec3 wrist_{0};
    std::array<dvec3, HAND_LANDMARK_COUNT> world_{};
    
    // Скорость (сглаженная)
    dvec3 smooth_vel_{0};
    dvec3 prev_pos_{0};
    bool has_prev_pos_ = false;
    
    // Калмановский фильтр для позиции
    KalmanFilter3D kf_{0.005, 0.05};
    
    // Жест
    HandGesture gesture_ = HandGesture::UNKNOWN;
    HandGesture prev_gesture_ = HandGesture::UNKNOWN;
    float pinch_strength_ = 0;
    float openness_ = 0;
    bool active_ = false;
    bool was_active_ = false;
    
    // Распознаватель жестов
    GestureRecognizer recognizer_;
    
    // Состояние взаимодействия
    HandInteractionState interaction_;
    
    // История позиций для сглаживания
    std::deque<dvec3> position_history_;
    static constexpr int HISTORY_SIZE = 5;
};

// ==================== Gesture Controller ====================

class GestureController {
public:
    GestureController();
    
    // Update both hands from tracker
    void update(HandTracker& tracker, const CameraIntrinsics& intrinsics, double dt);
    
    // Access
    VirtualHand& right_hand() { return right_hand_; }
    VirtualHand& left_hand() { return left_hand_; }
    const VirtualHand& right_hand() const { return right_hand_; }
    const VirtualHand& left_hand() const { return left_hand_; }
    
    // Калибровка
    bool is_calibrated() const { return calibrated_; }
    void calibrate();
    
    // Настройки
    void set_enabled(bool e) { enabled_ = e; }
    bool enabled() const { return enabled_; }
    
    // Callback на смену жеста
    std::function<void(VirtualHand&, HandGesture, HandGesture)> on_gesture_change;
    
private:
    VirtualHand right_hand_;
    VirtualHand left_hand_;
    bool calibrated_ = false;
    bool enabled_ = true;
    int calibration_frames_ = 0;
    static constexpr int CALIBRATION_FRAMES = 30;
};

} // namespace hlp
