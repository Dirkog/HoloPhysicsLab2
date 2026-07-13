#pragma once

// ============================================================
// HoloPhysics Lab 2 — Camera Capture Module
// Захват видео с веб-камеры в отдельном потоке
// Thread-safe double buffering, автоконвертация BGR→RGB
// ============================================================

#include <opencv2/opencv.hpp>
#include <atomic>
#include <thread>
#include <functional>
#include <chrono>

namespace hlp {

struct CameraFrame {
    cv::Mat bgr;             // Raw from camera (BGR)
    cv::Mat rgb;             // Converted for neural networks (RGB)
    int64_t timestamp_us;    // Микросекунды
    int frame_id = 0;        // Номер кадра
    bool valid = false;      // Флаг готовности
    int width = 0, height = 0;

    CameraFrame() = default;
    
    CameraFrame(const CameraFrame& other) {
        if (!other.bgr.empty()) other.bgr.copyTo(bgr);
        if (!other.rgb.empty()) other.rgb.copyTo(rgb);
        timestamp_us = other.timestamp_us;
        frame_id = other.frame_id;
        valid = other.valid;
        width = other.width;
        height = other.height;
    }
    
    CameraFrame& operator=(const CameraFrame& other) {
        if (this != &other) {
            if (!other.bgr.empty()) other.bgr.copyTo(bgr);
            if (!other.rgb.empty()) other.rgb.copyTo(rgb);
            timestamp_us = other.timestamp_us;
            frame_id = other.frame_id;
            valid = other.valid;
            width = other.width;
            height = other.height;
        }
        return *this;
    }
};

struct CameraIntrinsics {
    double fx = 615.0;   // Фокусное X (для 640×480)
    double fy = 615.0;   // Фокусное Y
    double cx = 320.0;   // Оптический центр X
    double cy = 240.0;   // Оптический центр Y
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0; // Distortion
};

class CameraCapture {
public:
    CameraCapture();
    ~CameraCapture();

    bool open(int camera_id = 0, 
              int width = 640, int height = 480, int fps = 60,
              bool mirror = true);
    void start();
    void stop();
    bool is_open() const { return cap_.isOpened(); }
    bool is_running() const { return running_.load(); }

    // Thread-safe получение последнего кадра (non-blocking)
    bool read_latest(CameraFrame& out);

    // Параметры камеры
    const CameraIntrinsics& intrinsics() const { return intrinsics_; }
    int width() const { return width_; }
    int height() const { return height_; }
    bool mirror() const { return mirror_; }

    // Callback для пайплайна (вызывается в потоке захвата)
    void set_frame_callback(std::function<void(const CameraFrame&)> cb) {
        callback_ = std::move(cb);
    }

    // Статистика
    double fps() const { return capture_fps_; }
    int total_frames() const { return frame_count_; }

private:
    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_{false};
    std::function<void(const CameraFrame&)> callback_;

    // Double buffering (lock-free)
    CameraFrame frames_[4];  // Quad buffering для безопасности
    std::atomic<int> write_idx_{0};
    std::atomic<int> read_idx_{0};

    CameraIntrinsics intrinsics_;
    int width_ = 640, height_ = 480, target_fps_ = 60;
    bool mirror_ = true;
    
    double capture_fps_ = 0;
    int frame_count_ = 0;

    void capture_loop();
};

} // namespace hlp
