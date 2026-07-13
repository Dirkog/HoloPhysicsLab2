#pragma once

// ============================================================
// HoloPhysics Lab 2 — IHandSource (Strategy Pattern)
//
// Абстракция источника данных о руках.
// Позволяет переключаться между:
//   - OpenXR (шлемы: Quest 3, Vision Pro, HoloLens)
//   - OpenCV + MediaPipe (веб-камера)
//   - Simulation (для отладки без железа)
//
// Единый формат данных. Фильтры (Kalman + Hermite) применяются
// ТОЛЬКО для OpenCV-источника. OpenXR даёт уже чистые данные.
// ============================================================

#include "../../core/Types.h"
#include "../../cv/HandTracker.h"  // For HandData struct (shared format)
#include <memory>
#include <string>

namespace hlp {

// ==================== Unified Hand Data ====================

struct UnifiedHandData {
    // 21 landmarks (MediaPipe-compatible format)
    std::array<HandLandmark, 21> landmarks;
    
    // 3D world positions
    std::array<dvec3, 21> world_positions;
    
    // Overall state
    dvec3 palm_center{0};
    dvec3 index_tip{0};
    dvec3 wrist{0};
    dvec3 velocity{0};
    float confidence = 0.0f;
    bool detected = false;
    int hand_id = 0;        // 0=right, 1=left
    int64_t timestamp_us = 0;
    double depth_m = 1.0;
    
    // Source type
    enum SourceType { UNKNOWN, OPENXR, OPENCV, SIMULATION };
    SourceType source = UNKNOWN;
    
    // OpenXR-specific (when available)
    bool is_controller_active = false;
    dvec3 controller_position{0};
    dquat controller_orientation{1, 0, 0, 0};
};

// ==================== Hand Source Interface ====================

class IHandSource {
public:
    virtual ~IHandSource() = default;
    
    // === Main update ===
    // Returns unified hand data for both hands
    virtual bool update(std::array<UnifiedHandData, 2>& hands) = 0;
    
    // === Properties ===
    virtual const char* name() const = 0;
    virtual bool is_available() const = 0;
    virtual int priority() const = 0;  // Higher = preferred
    
    // === Configuration ===
    virtual bool configure(const std::string& key, const std::string& value) { return false; }
};

// ==================== Source Implementations ====================

// Source 1: OpenXR
class OpenXRHandSource : public IHandSource {
public:
    OpenXRHandSource();
    ~OpenXRHandSource() = default;
    
    bool update(std::array<UnifiedHandData, 2>& hands) override;
    const char* name() const override { return "OpenXR"; }
    bool is_available() const override;
    int priority() const override { return 100; }  // Highest
    
private:
    // Reference to OpenXR manager (set externally)
    void* xr_manager_ = nullptr;
};

// Source 2: OpenCV + MediaPipe
class OpenCVHandSource : public IHandSource {
public:
    OpenCVHandSource();
    ~OpenCVHandSource() = default;
    
    bool update(std::array<UnifiedHandData, 2>& hands) override;
    const char* name() const override { return "OpenCV"; }
    bool is_available() const override;
    int priority() const override { return 50; }
    
    // Access to underlying components for configuration
    class HandTracker* tracker() { return tracker_.get(); }
    class CameraCapture* camera() { return camera_.get(); }

private:
    std::unique_ptr<class CameraCapture> camera_;
    std::unique_ptr<class HandTracker> tracker_;
    bool initialized_ = false;
};

// Source 3: Simulation (for debug/development)
class SimulationHandSource : public IHandSource {
public:
    SimulationHandSource();
    ~SimulationHandSource() = default;
    
    bool update(std::array<UnifiedHandData, 2>& hands) override;
    const char* name() const override { return "Simulation"; }
    bool is_available() const override { return true; }
    int priority() const override { return 10; }  // Lowest
    
    void set_gesture_cycle(const std::string& cycle); // "open_pinch_fist_point"

private:
    double sim_time_ = 0;
    int gesture_phase_ = 0;
};

// ==================== Hand Source Manager ====================

class HandSourceManager {
public:
    HandSourceManager();
    ~HandSourceManager();
    
    // === Registration ===
    void register_source(std::unique_ptr<IHandSource> source);
    void unregister_source(const std::string& name);
    
    // === Auto-select best source ===
    IHandSource* select_best_source();
    IHandSource* current_source() const { return current_; }
    
    // === Get unified hand data ===
    bool get_hands(std::array<UnifiedHandData, 2>& hands);
    
    // === Force specific source ===
    bool set_source(const std::string& name);
    
    // === List available sources ===
    std::vector<std::string> available_sources() const;
    
    // === Processing chain (for OpenCV source only) ===
    void set_processing_chain(bool kalman, bool hermite, bool mahalanobis) {
        use_kalman_ = kalman;
        use_hermite_ = hermite;
        use_mahalanobis_ = mahalanobis;
    }

private:
    std::vector<std::unique_ptr<IHandSource>> sources_;
    IHandSource* current_ = nullptr;
    
    // Processing flags (only applied to OpenCV source)
    bool use_kalman_ = true;
    bool use_hermite_ = true;
    bool use_mahalanobis_ = true;
    
    void apply_filtering(std::array<UnifiedHandData, 2>& hands);
};

} // namespace hlp
