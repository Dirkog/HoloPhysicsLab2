#include "IHandSource.h"
#include "../../cv/CameraCapture.h"
#include "../../cv/HandTracker.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace hlp {

// ==================== OpenCVHandSource ====================

OpenCVHandSource::OpenCVHandSource() = default;

bool OpenCVHandSource::is_available() const {
    return initialized_;
}

bool OpenCVHandSource::update(std::array<UnifiedHandData, 2>& hands) {
    if (!initialized_) {
        // Lazy init
        camera_ = std::make_unique<CameraCapture>();
        if (!camera_->open(0, 640, 480, 60)) {
            return false;
        }
        
        tracker_ = std::make_unique<HandTracker>();
        tracker_->init(HandTrackerMode::SIMULATION);
        camera_->start();
        initialized_ = true;
    }
    
    // Get camera frame
    CameraFrame frame;
    if (!camera_->read_latest(frame)) {
        return false;
    }
    
    // Run hand tracking
    if (!tracker_->process_frame(frame)) {
        return false;
    }
    
    // Convert to unified format
    for (int i = 0; i < tracker_->hand_count() && i < 2; i++) {
        auto& raw = tracker_->hand(i);
        auto& unified = hands[i];
        
        // Unproject to 3D
        tracker_->unproject_to_3d(raw, camera_->intrinsics());
        
        // Copy landmarks
        for (int j = 0; j < 21; j++) {
            unified.landmarks[j] = raw.landmarks[j];
            unified.world_positions[j] = dvec3(
                raw.world_positions[j].x,
                raw.world_positions[j].y,
                raw.world_positions[j].z
            );
        }
        
        // Key points
        unified.palm_center = dvec3(0);
        for (int j : {5, 9, 13, 17}) 
            unified.palm_center += unified.world_positions[j];
        unified.palm_center /= 4;
        
        unified.index_tip = unified.world_positions[8];
        unified.wrist = unified.world_positions[0];
        unified.confidence = raw.confidence;
        unified.detected = raw.detected;
        unified.hand_id = raw.hand_id;
        unified.depth_m = raw.depth_m;
        unified.source = UnifiedHandData::OPENCV;
    }
    
    return true;
}

// ==================== SimulationHandSource ====================

SimulationHandSource::SimulationHandSource() = default;

bool SimulationHandSource::update(std::array<UnifiedHandData, 2>& hands) {
    sim_time_ += 0.016;  // ~60fps simulation
    
    // Lissajous hand trajectory
    double t = sim_time_;
    double hand_x = 320 + 150 * std::sin(t * 0.5);
    double hand_y = 240 + 100 * std::sin(t * 0.7 + 1.0);
    double palm_w = 120 + 30 * std::sin(t * 0.3);
    double scale = palm_w / 120.0;
    
    // Gesture cycle: 0-5s OPEN, 5-8s PINCH, 8-12s FIST, 12-15s POINT
    double phase = std::fmod(t, 15.0);
    double thumb_curl = 1.0, index_curl = 1.0, other_curl = 1.0;
    
    if (phase < 5.0) { /* OPEN */ }
    else if (phase < 8.0) { thumb_curl = 0.3; index_curl = 0.3; }
    else if (phase < 12.0) { thumb_curl = 0.5; index_curl = 0.2; other_curl = 0.2; }
    else { thumb_curl = 0.5; other_curl = 0.3; }
    
    for (int h = 0; h < 2; h++) {
        auto& hand = hands[h];
        double ox = h == 0 ? hand_x : 640 - hand_x;
        double oy = hand_y + h * 60;
        
        auto set_lm = [&](int idx, double dx, double dy, double dz = 0) {
            hand.landmarks[idx].x = ox + dx * scale;
            hand.landmarks[idx].y = oy + dy * scale;
            hand.landmarks[idx].z = dz * scale * 0.01;
            hand.landmarks[idx].visibility = 1.0f;
            hand.world_positions[idx] = dvec3(ox + dx * scale, oy + dy * scale, dz);
        };
        
        set_lm(0, 0, 0);
        set_lm(1, -20, -10, 0.01);
        set_lm(5, 10, -15, 0);
        set_lm(9, 25, -18, 0);
        set_lm(13, 40, -16, 0);
        set_lm(17, 55, -10, 0);
        
        // Fingers with gesture-based curl
        auto finger = [&](int mcp, int dx, int dy, double curl) {
            double len = 40 * scale;
            set_lm(mcp+1, dx*len*curl, dy*len*curl - (1-curl)*15, 0.02);
            set_lm(mcp+2, dx*len*curl*0.7, dy*len*curl*0.7 - (1-curl)*10, 0.03);
            set_lm(mcp+3, dx*len*curl*0.5, dy*len*curl*0.5 - (1-curl)*5, 0.04);
        };
        
        finger(1, -0.8, -0.5, thumb_curl);  // Thumb
        finger(5, 0, -1, index_curl);        // Index
        finger(9, 0, -1, other_curl);        // Middle
        finger(13, 0.05, -0.95, other_curl); // Ring
        finger(17, 0.1, -0.9, other_curl);   // Pinky
        
        // Palm center
        hand.palm_center = dvec3(0);
        for (int j : {5, 9, 13, 17}) 
            hand.palm_center += hand.world_positions[j];
        hand.palm_center /= 4;
        
        hand.index_tip = hand.world_positions[8];
        hand.wrist = hand.world_positions[0];
        hand.confidence = 0.95f;
        hand.detected = true;
        hand.hand_id = h;
        hand.source = UnifiedHandData::SIMULATION;
    }
    
    return true;
}

// ==================== HandSourceManager ====================

HandSourceManager::HandSourceManager() {
    // Register default sources
    register_source(std::make_unique<SimulationHandSource>());
    register_source(std::make_unique<OpenCVHandSource>());
}

HandSourceManager::~HandSourceManager() = default;

void HandSourceManager::register_source(std::unique_ptr<IHandSource> source) {
    if (source) {
        std::cout << "[HandSource] Registered: " << source->name() << std::endl;
        sources_.push_back(std::move(source));
    }
}

void HandSourceManager::unregister_source(const std::string& name) {
    sources_.erase(
        std::remove_if(sources_.begin(), sources_.end(),
            [&](const auto& s) { return s->name() == name; }),
        sources_.end());
}

IHandSource* HandSourceManager::select_best_source() {
    // Sort by priority, pick highest available
    std::sort(sources_.begin(), sources_.end(),
        [](const auto& a, const auto& b) {
            return a->priority() > b->priority();
        });
    
    for (auto& s : sources_) {
        if (s->is_available()) {
            current_ = s.get();
            std::cout << "[HandSource] Selected: " << s->name() << std::endl;
            return current_;
        }
    }
    
    // Fallback to simulation (always available)
    current_ = sources_.back().get();
    return current_;
}

bool HandSourceManager::get_hands(std::array<UnifiedHandData, 2>& hands) {
    if (!current_) select_best_source();
    if (!current_) return false;
    
    bool result = current_->update(hands);
    
    // Apply filtering only for OpenCV source
    if (use_kalman_ || use_mahalanobis_) {
        apply_filtering(hands);
    }
    
    return result;
}

bool HandSourceManager::set_source(const std::string& name) {
    for (auto& s : sources_) {
        if (s->name() == name && s->is_available()) {
            current_ = s.get();
            return true;
        }
    }
    return false;
}

std::vector<std::string> HandSourceManager::available_sources() const {
    std::vector<std::string> names;
    for (const auto& s : sources_) {
        names.push_back(s->name());
    }
    return names;
}

void HandSourceManager::apply_filtering(std::array<UnifiedHandData, 2>& hands) {
    // Kalman + Mahalanobis filtering applied here
    // (In production: integrate with existing PredictiveKalman)
    (void)hands;
}

// ==================== OpenXRHandSource ====================

OpenXRHandSource::OpenXRHandSource() = default;

bool OpenXRHandSource::is_available() const {
    // Check if OpenXR runtime is available
    // (in production: check xrCreateInstance result)
    return false;  // Will be set externally when XR is initialized
}

bool OpenXRHandSource::update(std::array<UnifiedHandData, 2>& hands) {
    // OpenXR provides hand data via XrHandJointLocationEXT
    // This would convert from XR format to UnifiedHandData
    // For now: return false (will use fallback)
    (void)hands;
    return false;
}

} // namespace hlp
