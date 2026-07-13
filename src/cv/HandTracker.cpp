#include "HandTracker.h"
#include <random>
#include <algorithm>
#include <iostream>

namespace hlp {

// ==================== HandData ====================

float HandData::palm_width() const {
    return finger_distance(HAND_INDEX_MCP, HAND_PINKY_MCP);
}

float HandData::finger_distance(int tip_a, int tip_b) const {
    float dx = landmarks[tip_a].x - landmarks[tip_b].x;
    float dy = landmarks[tip_a].y - landmarks[tip_b].y;
    return std::sqrt(dx * dx + dy * dy);
}

bool HandData::is_finger_extended(int tip_idx, int pip_idx) const {
    // In image space: finger is extended if tip.y < pip.y (higher on screen)
    // In our coordinate system: tip_z < pip_z (closer to camera = more extended)
    return landmarks[tip_idx].y < landmarks[pip_idx].y - 0.02f;
}

// ==================== HandTracker ====================

HandTracker::HandTracker() {
    hands_.resize(2); // Max 2 hands
}

HandTracker::~HandTracker() {
    shutdown();
}

bool HandTracker::init(HandTrackerMode mode, const char* model_path) {
    mode_ = mode;

    switch (mode) {
        case HandTrackerMode::MEDIAPIPE:
            if (!init_mediapipe(model_path)) {
                std::cerr << "[HandTracker] MediaPipe init failed, falling back to simulation" << std::endl;
                mode_ = HandTrackerMode::SIMULATION;
                return init_mediapipe(nullptr);
            }
            break;
        case HandTrackerMode::ONNX_MODEL:
            if (!init_onnx(model_path)) {
                std::cerr << "[HandTracker] ONNX init failed, falling back to simulation" << std::endl;
                mode_ = HandTrackerMode::SIMULATION;
            }
            break;
        case HandTrackerMode::SIMULATION:
            std::cout << "[HandTracker] Simulation mode active" << std::endl;
            break;
        case HandTrackerMode::DISABLED:
            return true;
    }

    // Init smoothing buffers
    for (int h = 0; h < 2; h++) {
        smoothed_positions_[h].resize(HAND_LANDMARK_COUNT);
        for (int i = 0; i < HAND_LANDMARK_COUNT; i++) {
            smoothed_positions_[h][i] = {0, 0, 0};
        }
    }

    initialized_ = true;
    return true;
}

void HandTracker::shutdown() {
    initialized_ = false;

    if (mediapipe_graph_) {
        // mediapipe_graph_->Close();
        mediapipe_graph_ = nullptr;
    }
    if (onnx_session_) {
        // onnx_session_.reset();
        onnx_session_ = nullptr;
    }
}

bool HandTracker::init_mediapipe(const char* model_path) {
    // MediaPipe C++ initialization:
    //
    // 1. Load calculator graph config from binarypb
    // 2. Initialize: mp::CalculatorGraph graph;
    // 3. graph.Initialize(config);
    // 4. Add input stream: "input_video"
    // 5. Add output streams: "hand_landmarks", "handedness"
    // 6. graph.StartRun({});
    //
    // For now: return false to trigger simulation fallback
    (void)model_path;
    return false;
}

bool HandTracker::init_onnx(const char* model_path) {
    // ONNX Runtime initialization:
    //
    // 1. Ort::Env env;
    // 2. Ort::SessionOptions opts;
    // 3. Ort::Session session(env, model_path, opts);
    // 4. Get input/output names
    //
    (void)model_path;
    return false;
}

bool HandTracker::process_frame(const CameraFrame& frame) {
    if (!initialized_ || mode_ == HandTrackerMode::DISABLED) return false;

    switch (mode_) {
        case HandTrackerMode::MEDIAPIPE:
            return process_mediapipe(frame);
        case HandTrackerMode::SIMULATION:
            return process_simulation(frame);
        default:
            return process_simulation(frame);
    }
}

// ==================== Simulation Mode ====================

bool HandTracker::process_simulation(const CameraFrame& frame) {
    sim_time_ += 0.016; // ~60fps

    // Имитация движения руки: фигура Лиссажу
    double t = sim_time_;
    double hand_x = 320 + 150 * std::sin(t * 0.5);
    double hand_y = 240 + 100 * std::sin(t * 0.7 + 1.0);
    double hand_z = 0;

    double palm_w = 120 + 30 * std::sin(t * 0.3); // "приближение/отдаление"

    // Жесты по циклу: 0-5s = OPEN, 5-8s = PINCH, 8-12s = FIST, 12-15s = POINT
    double phase = std::fmod(t, 15.0);

    for (int h = 0; h < 2; h++) {
        auto& hand = hands_[h];
        double offset_x = h == 0 ? hand_x : 640 - hand_x;
        double offset_y = hand_y + h * 50;
        double scale = palm_w / 120.0;

        // Генерация 21 landmark на основе простой кинематической модели
        auto set_lm = [&](int idx, double dx, double dy, double dz = 0, double vis = 1.0) {
            hand.landmarks[idx].x = offset_x + dx * scale;
            hand.landmarks[idx].y = offset_y + dy * scale;
            hand.landmarks[idx].z = hand_z + dz * scale * 0.01;
            hand.landmarks[idx].visibility = vis;
        };

        // Palm base
        set_lm(HAND_WRIST, 0, 0, 0);
        set_lm(HAND_THUMB_CMC, -20, -10, 0.01);
        set_lm(HAND_INDEX_MCP, 10, -15, 0);
        set_lm(HAND_MIDDLE_MCP, 25, -18, 0);
        set_lm(HAND_RING_MCP, 40, -16, 0);
        set_lm(HAND_PINKY_MCP, 55, -10, 0);

        // Fingers - length based on gesture
        double thumb_curl = 1.0;
        double index_curl = 1.0;
        double middle_curl = 1.0;
        double ring_curl = 1.0;
        double pinky_curl = 1.0;

        if (phase < 5.0) {
            // OPEN PALM: all extended
        } else if (phase < 8.0) {
            // PINCH: thumb + index curled, others extended
            thumb_curl = 0.3; index_curl = 0.3;
        } else if (phase < 12.0) {
            // FIST: all curled
            thumb_curl = 0.5; index_curl = 0.2; middle_curl = 0.2;
            ring_curl = 0.2; pinky_curl = 0.2;
        } else {
            // POINTING: only index extended
            thumb_curl = 0.5; index_curl = 1.0; middle_curl = 0.3;
            ring_curl = 0.3; pinky_curl = 0.3;
        }

        auto make_finger = [&](int mcp, int pip, int dip, int tip,
                                double dx, double dy, double curl) {
            double len = 40 * scale;
            set_lm(pip, dx * len * curl, dy * len * curl - (1-curl) * 15, 0.02);
            set_lm(dip, dx * len * curl * 0.7, dy * len * curl * 0.7 - (1-curl) * 10, 0.03);
            set_lm(tip, dx * len * curl * 0.5, dy * len * curl * 0.5 - (1-curl) * 5, 0.04);
        };

        // Thumb (goes sideways)
        make_finger(HAND_THUMB_MCP, HAND_THUMB_IP, HAND_THUMB_IP, HAND_THUMB_TIP,
                    0.5, -0.8, thumb_curl);

        // Index (upward in image = negative y)
        make_finger(HAND_INDEX_MCP, HAND_INDEX_PIP, HAND_INDEX_DIP, HAND_INDEX_TIP,
                    0, -1, index_curl);

        // Middle
        make_finger(HAND_MIDDLE_MCP, HAND_MIDDLE_PIP, HAND_MIDDLE_DIP, HAND_MIDDLE_TIP,
                    0, -1, middle_curl);

        // Ring
        make_finger(HAND_RING_MCP, HAND_RING_PIP, HAND_RING_DIP, HAND_RING_TIP,
                    0.05, -0.95, ring_curl);

        // Pinky
        make_finger(HAND_PINKY_MCP, HAND_PINKY_PIP, HAND_PINKY_DIP, HAND_PINKY_TIP,
                    0.1, -0.9, pinky_curl);

        hand.confidence = 0.95f;
        hand.detected = true;
        hand.hand_id = h;
        hand.timestamp_us = 0;

        // Применить сглаживание
        apply_smoothing(hand);
    }

    detected_hands_ = 2;

    // Callback
    if (hand_callback_) {
        for (int i = 0; i < detected_hands_; i++) {
            hand_callback_(hands_[i]);
        }
    }

    return true;
}

// ==================== MediaPipe Processing ====================

bool HandTracker::process_mediapipe(const CameraFrame& frame) {
    // MediaPipe C++ processing:
    //
    // 1. auto packet = mp::MakePacket<cv::Mat>(frame.rgb).At(mp::Timestamp::PostStream());
    // 2. graph.AddPacketToInputStream("input_video", packet);
    //
    // 3. Get output packets:
    //    auto landmarks_packet = graph.GetOutputPacket("hand_landmarks");
    //    auto handedness_packet = graph.GetOutputPacket("handedness");
    //
    // 4. Parse landmarks into hands_ array
    //
    // For now: fallback to simulation
    return process_simulation(frame);
}

// ==================== 2D → 3D Unprojection ====================

void HandTracker::unproject_to_3d(HandData& hand, const CameraIntrinsics& intrinsics,
                                   double known_palm_width) {
    if (!hand.detected) return;

    // Step 1: Estimate depth from palm width
    float palm_px = hand.palm_width();
    if (palm_px < 1.0f) palm_px = 1.0f;

    // Z = (known_width_m * focal_x) / palm_width_px
    double depth = (known_palm_width * intrinsics.fx) / palm_px;
    depth = std::clamp(depth, 0.1, 2.0); // 10cm to 2m
    hand.depth_m = depth;

    // Step 2: Unproject each point
    for (int i = 0; i < HAND_LANDMARK_COUNT; i++) {
        auto& lm = hand.landmarks[i];
        auto& wp = hand.world_positions[i];

        // Pinhole camera model (mirrored for natural interaction)
        // X = (u - cx) * Z / fx
        // Y = (v - cy) * Z / fy
        // Then mirror X and invert Y for physics coordinate system
        double x = -(lm.x - intrinsics.cx) * depth / intrinsics.fx;  // Mirror X
        double y = -(lm.y - intrinsics.cy) * depth / intrinsics.fy;  // Invert Y
        double z = depth + lm.z * 0.01;  // MediaPipe z-offset

        wp.x = x;
        wp.y = y;
        wp.z = z;
    }
}

// ==================== Smoothing ====================

void HandTracker::apply_smoothing(HandData& hand) {
    double alpha = smoothing_alpha_;
    int hid = hand.hand_id;

    if (hid < 0 || hid > 1) return;

    for (int i = 0; i < HAND_LANDMARK_COUNT; i++) {
        auto& sm = smoothed_positions_[hid][i];
        double& raw_x = hand.landmarks[i].x;
        double& raw_y = hand.landmarks[i].y;
        double& raw_z = hand.landmarks[i].z;

        // First frame: initialize
        if (sm[0] == 0 && sm[1] == 0 && sm[2] == 0) {
            sm[0] = raw_x;
            sm[1] = raw_y;
            sm[2] = raw_z;
        }

        // EMA filter
        sm[0] = sm[0] * alpha + raw_x * (1.0 - alpha);
        sm[1] = sm[1] * alpha + raw_y * (1.0 - alpha);
        sm[2] = sm[2] * alpha + raw_z * (1.0 - alpha);

        // Store smoothed values back (slightly, for stability)
        raw_x = raw_x * 0.3 + sm[0] * 0.7;
        raw_y = raw_y * 0.3 + sm[1] * 0.7;
        raw_z = raw_z * 0.3 + sm[2] * 0.7;
    }
}

const HandData& HandTracker::hand(int idx) const {
    return hands_[idx % 2];
}

HandData& HandTracker::hand(int idx) {
    return hands_[idx % 2];
}

} // namespace hlp
