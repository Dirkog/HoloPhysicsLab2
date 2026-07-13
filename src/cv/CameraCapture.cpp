#include "CameraCapture.h"
#include <chrono>
#include <thread>

namespace hlp {

CameraCapture::CameraCapture() = default;

CameraCapture::~CameraCapture() {
    stop();
}

bool CameraCapture::open(int camera_id, int width, int height, int fps, bool mirror) {
    width_ = width;
    height_ = height;
    target_fps_ = fps;
    mirror_ = mirror;

    cap_.open(camera_id, cv::CAP_ANY);
    if (!cap_.isOpened()) {
        // Попробовать DirectShow (Windows)
        cap_.open(camera_id, cv::CAP_DSHOW);
    }
    if (!cap_.isOpened()) return false;

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, fps);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Минимальный буфер

    // Инициализация intrinsic параметров
    intrinsics_.fx = width * 0.95;
    intrinsics_.fy = intrinsics_.fx;
    intrinsics_.cx = width * 0.5;
    intrinsics_.cy = height * 0.5;

    return true;
}

void CameraCapture::start() {
    if (running_.load()) return;
    running_.store(true);
    capture_thread_ = std::thread(&CameraCapture::capture_loop, this);
}

void CameraCapture::stop() {
    running_.store(false);
    if (capture_thread_.joinable()) capture_thread_.join();
    if (cap_.isOpened()) cap_.release();
}

void CameraCapture::capture_loop() {
    cv::Mat frame_bgr, frame_rgb;
    int frame_id = 0;
    auto last_time = std::chrono::steady_clock::now();
    int fps_counter = 0;
    double fps_accum = 0;

    while (running_.load()) {
        // Захват кадра
        if (!cap_.read(frame_bgr)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        if (frame_bgr.empty()) continue;

        // Зеркалирование (как в Zoom/селфи)
        if (mirror_) {
            cv::flip(frame_bgr, frame_bgr, 1);
        }

        // Конвертация BGR → RGB для MediaPipe/ONNX
        cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);

        // Заполняем фрейм
        CameraFrame frame;
        frame_bgr.copyTo(frame.bgr);
        frame_rgb.copyTo(frame.rgb);
        frame.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        frame.frame_id = frame_id++;
        frame.valid = true;
        frame.width = frame_bgr.cols;
        frame.height = frame_bgr.rows;

        // Quad buffering: пишем в следующий слот
        int w_idx = write_idx_.load();
        int next_w_idx = (w_idx + 1) % 4;
        frames_[next_w_idx] = frame;
        write_idx_.store(next_w_idx);
        read_idx_.store(next_w_idx);  // Сигнал читателю

        // Callback (для немедленной обработки)
        if (callback_) {
            callback_(frame);
        }

        // FPS подсчёт
        fps_counter++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_time).count();
        if (elapsed >= 1.0) {
            capture_fps_ = fps_counter / elapsed;
            fps_counter = 0;
            last_time = now;
        }

        frame_count_++;

        // CPU-friendly ожидание
        double frame_time = 1.0 / target_fps_;
        double work_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - now).count();
        if (work_time < frame_time) {
            std::this_thread::sleep_for(
                std::chrono::microseconds((int)((frame_time - work_time) * 1e6)));
        }
    }
}

bool CameraCapture::read_latest(CameraFrame& out) {
    int r_idx = read_idx_.load();
    int w_idx = write_idx_.load();
    
    if (r_idx == w_idx && !frames_[r_idx].valid)
        return false;

    out = frames_[r_idx];
    return out.valid;
}

} // namespace hlp
