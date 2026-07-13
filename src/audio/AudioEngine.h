#pragma once

// ============================================================
// HoloPhysics Lab 2 — Physical Audio Engine
//
// Процедурная генерация звука столкновений и взаимодействий.
// Spatial audio (HRTF) для 3D позиционирования.
// Использует miniaudio (единственный заголовочный файл).
// ============================================================

#define MINIAUDIO_IMPLEMENTATION
#include "../core/Types.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <atomic>
#include <cmath>

namespace hlp {

// ==================== Audio Configuration ====================

struct AudioConfig {
    bool enabled = true;
    float master_volume = 0.8f;
    float sfx_volume = 1.0f;
    float spatial_blend = 0.7f;     // [0,1] 0=mono, 1=full spatial
    float doppler_factor = 1.0f;
    int sample_rate = 48000;
    
    // Procedural synthesis
    float collision_volume_scale = 0.5f;   // Volume = scale * (impulse / max_impulse)
    float collision_freq_min = 200.0f;      // Hz for low mass (wood, plastic)
    float collision_freq_max = 2000.0f;     // Hz for high mass (metal, glass)
    float collision_decay = 0.3f;            // Seconds
};

// ==================== Collision Sound Generator ====================

class CollisionSound {
public:
    CollisionSound() = default;
    
    // Generate collision sound based on physics parameters
    // Returns: pointer to audio buffer (mono float32)
    void generate(double impulse, double mass_a, double mass_b,
                   double restitution, double material_a, double material_b);
    
    const std::vector<float>& buffer() const { return buffer_; }
    int sample_rate() const { return 48000; }
    float duration() const { return duration_; }

private:
    std::vector<float> buffer_;
    float duration_ = 0.3f;
    
    // Material properties for frequency calculation
    struct MaterialProps {
        const char* name;
        float freq_mult;    // Frequency multiplier
        float decay_mult;   // Decay multiplier
        float brightness;   // High-frequency content
    };
    
    static constexpr MaterialProps materials_[4] = {
        {"metal",    2.0f,  0.8f, 0.9f},   // Bright, long ring
        {"wood",     0.5f,  0.5f, 0.3f},   // Warm, short
        {"plastic",  0.8f,  0.6f, 0.5f},   // Medium
        {"rubber",   0.3f,  0.3f, 0.1f},   // Dull, short
    };
    
    // Frequency modulation for realistic impact sound
    float fm_synthesis(float t, float freq, float decay) const;
};

// ==================== Audio Source (3D Spatial) ====================

struct AudioSource {
    dvec3 position{0};
    dvec3 velocity{0};
    std::shared_ptr<CollisionSound> sound;
    float volume = 1.0f;
    float pitch = 1.0f;
    float pan = 0.0f;           // -1=left, 0=center, 1=right
    float distance_atten = 1.0f;
    bool playing = false;
    float play_time = 0;
    float duration = 0;
    int source_id = -1;
};

// ==================== Audio Engine ====================

class AudioEngine {
public:
    AudioEngine();
    ~AudioEngine();

    // === Lifecycle ===
    bool init(const AudioConfig& config);
    void shutdown();
    bool is_initialized() const { return initialized_; }

    // === Playback ===
    int play_collision(const dvec3& position, const dvec3& velocity,
                       double impulse, double mass_a, double mass_b,
                       double restitution);
    
    int play_sound(const dvec3& position, std::shared_ptr<CollisionSound> sound,
                    float volume = 1.0f);
    
    void stop_source(int source_id);
    void stop_all();

    // === Per-frame update ===
    void update(const dvec3& listener_pos, const dvec3& listener_vel,
                const dvec3& listener_forward, const dvec3& listener_up,
                double dt);

    // === Listener ===
    void set_listener_pose(const dvec3& pos, const dvec3& forward,
                           const dvec3& up, const dvec3& vel = dvec3(0));

    // === Configuration ===
    void set_config(const AudioConfig& cfg) { config_ = cfg; }
    AudioConfig& config() { return config_; }

private:
    AudioConfig config_;
    bool initialized_ = false;
    
    // Active sources
    std::vector<AudioSource> active_sources_;
    int next_source_id_ = 1;
    
    // Internal audio thread callback
    static void audio_callback(void* userdata, float* output, int frame_count);
    void mix_output(float* output, int frame_count);
    
    // miniaudio engine
    void* engine_ = nullptr;     // ma_engine*
    void* sound_context_ = nullptr; // ma_sound_context
    
    // Utility
    float calculate_distance_atten(const dvec3& source_pos, const dvec3& listener_pos);
    float calculate_doppler(const dvec3& source_vel, const dvec3& listener_vel,
                            const dvec3& source_to_listener);
};

} // namespace hlp
