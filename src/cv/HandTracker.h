#pragma once

// ============================================================
// HoloPhysics Lab 2 — Hand Tracking Module
// MediaPipe Hands C++ API + Simulation fallback
// 21 landmarks per hand, 2 hands max
// ============================================================

#include <vector>
#include <array>
#include <cmath>
#include <memory>
#include <functional>
#include "CameraCapture.h"

namespace hlp {

// MediaPipe hand landmark indices
enum HandLandmarkIdx : int {
    HAND_WRIST = 0,
    HAND_THUMB_CMC, HAND_THUMB_MCP, HAND_THUMB_IP, HAND_THUMB_TIP,
    HAND_INDEX_MCP, HAND_INDEX_PIP, HAND_INDEX_DIP, HAND_INDEX_TIP,
    HAND_MIDDLE_MCP, HAND_MIDDLE_PIP, HAND_MIDDLE_DIP, HAND_MIDDLE_TIP,
    HAND_RING_MCP, HAND_RING_PIP, HAND_RING_DIP, HAND_RING_TIP,
    HAND_PINKY_MCP, HAND_PINKY_PIP, HAND_PINKY_DIP, HAND_PINKY_TIP,
    HAND_LANDMARK_COUNT = 21
};

struct HandLandmark {
    float x = 0, y = 0, z = 0;  // 2.5D: (px, px, depth)
    float visibility = 0.0f;
};

struct HandData {
    std::array<HandLandmark, HAND_LANDMARK_COUNT> landmarks;
    float confidence = 0.0f;
    bool detected = false;
    int hand_id = 0;       // 0 = right, 1 = left
    int64_t timestamp_us = 0;

    // 3D позиции в мировом пространстве (unprojected)
    struct World3D {
        double x = 0, y = 0, z = 0;
    };
    std::array<World3D, HAND_LANDMARK_COUNT> world_positions;
    double depth_m = 1.0;  // Estimated depth in meters

    // Convience accessors
    const HandLandmark& operator[](int i) const { return landmarks[i]; }
    HandLandmark& operator[](int i) { return landmarks[i]; }

    // Полезные вычисления
    float palm_width() const;
    float finger_distance(int tip_a, int tip_b) const;
    bool is_finger_extended(int tip_idx, int pip_idx) const;
};

// Режимы работы трекера
enum class HandTrackerMode {
    MEDIAPIPE,      // Реальный MediaPipe (требует установки)
    ONNX_MODEL,     // ONNX модель (требует .onnx файла)
    SIMULATION,     // Симуляция (для отладки без камеры)
    DISABLED        // Отключён
};

class HandTracker {
public:
    HandTracker();
    ~HandTracker();

    // Инициализация
    bool init(HandTrackerMode mode = HandTrackerMode::SIMULATION,
              const char* model_path = nullptr);
    void shutdown();

    // Основной метод: обработка кадра
    bool process_frame(const CameraFrame& frame);

    // Результаты
    int hand_count() const { return (int)detected_hands_; }
    const HandData& hand(int idx) const;
    HandData& hand(int idx);

    // 2D → 3D unprojection
    void unproject_to_3d(HandData& hand, const CameraIntrinsics& intrinsics,
                         double known_palm_width = 0.085);

    // Калибровка
    void set_intrinsics(const CameraIntrinsics& intrinsics) {
        intrinsics_ = intrinsics;
    }

    // Управление
    void set_mode(HandTrackerMode mode) { mode_ = mode; }
    HandTrackerMode mode() const { return mode_; }
    bool is_initialized() const { return initialized_; }

    // Сглаживание (EMA) для каждой точки
    void set_smoothing_alpha(double alpha) { smoothing_alpha_ = alpha; }
    double smoothing_alpha() const { return smoothing_alpha_; }

    // Callback
    void set_hand_detected_callback(std::function<void(const HandData&)> cb) {
        hand_callback_ = std::move(cb);
    }

private:
    HandTrackerMode mode_ = HandTrackerMode::SIMULATION;
    bool initialized_ = false;

    // Результаты
    std::vector<HandData> hands_;
    int detected_hands_ = 0;

    // Для MediaPipe: указатель на graph
    void* mediapipe_graph_ = nullptr;

    // Для ONNX: указатель на сессию
    void* onnx_session_ = nullptr;

    // Параметры камеры
    CameraIntrinsics intrinsics_;

    // Сглаживание
    double smoothing_alpha_ = 0.3;
    std::vector<std::array<double, 3>> smoothed_positions_[2]; // [hand][landmark]

    // Симуляция
    double sim_time_ = 0;

    // Внутренние методы
    bool init_mediapipe(const char* model_path);
    bool init_onnx(const char* model_path);
    bool process_mediapipe(const CameraFrame& frame);
    bool process_simulation(const CameraFrame& frame);

    // Фильтрация шума
    void apply_smoothing(HandData& hand);
    void apply_temporal_filter(HandData& hand);

    // Сброс smoothed позиций при потере руки
    void reset_smoothing(int hand_id);

    // Callback
    std::function<void(const HandData&)> hand_callback_;
};

} // namespace hlp
