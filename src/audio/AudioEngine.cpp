#include "AudioEngine.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <cstring>

// Include miniaudio (single-header library)
#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

namespace hlp {

// ==================== CollisionSound ====================

float CollisionSound::fm_synthesis(float t, float freq, float decay) const {
    // FM synthesis: carrier + modulation for metallic/natural sound
    float carrier = std::sin(2.0f * M_PI * freq * t);
    float modulator = std::sin(2.0f * M_PI * freq * 0.5f * t);
    float fm = std::sin(2.0f * M_PI * freq * t + modulator * 2.0f);
    
    // Mix carrier and FM
    float signal = carrier * 0.4f + fm * 0.6f;
    
    // Envelope: exponential decay
    float envelope = std::exp(-decay * t * 10.0f);
    
    // Initial transient (impact click)
    float click = std::exp(-t * 100.0f) * 0.8f;
    
    // Noise component (for texture)
    float noise = (float)(rand() % 1000) / 1000.0f * 2.0f - 1.0f;
    noise *= std::exp(-t * 50.0f) * 0.1f;
    
    return (signal * envelope + click + noise) * 0.3f;
}

void CollisionSound::generate(double impulse, double mass_a, double mass_b,
                               double restitution, double mat_a, double mat_b) {
    // Determine material properties
    int ma = (int)mat_a % 4;
    int mb = (int)mat_b % 4;
    
    float freq_mult = (materials_[ma].freq_mult + materials_[mb].freq_mult) * 0.5f;
    float decay_mult = (materials_[ma].decay_mult + materials_[mb].decay_mult) * 0.5f;
    
    // Base frequency from collision
    double rel_mass = (mass_a * mass_b) / (mass_a + mass_b);
    float base_freq = 200.0f + (float)(impulse * freq_mult * 10.0f);
    base_freq = std::clamp(base_freq, config_.collision_freq_min, config_.collision_freq_max);
    
    // Duration and decay
    float decay = 1.0f / (config_.collision_decay * decay_mult);
    float duration_sec = config_.collision_decay * (1.0f + restitution) * decay_mult;
    
    // Generate buffer
    int num_samples = (int)(sample_rate() * duration_sec);
    buffer_.resize(num_samples);
    
    for (int i = 0; i < num_samples; i++) {
        float t = (float)i / sample_rate();
        buffer_[i] = fm_synthesis(t, base_freq, decay);
        
        // Add harmonics
        buffer_[i] += fm_synthesis(t, base_freq * 2.0f, decay * 0.8f) * 0.3f;
        buffer_[i] += fm_synthesis(t, base_freq * 0.5f, decay * 1.2f) * 0.2f;
        
        // Normalize
        buffer_[i] *= 0.6f;
    }
    
    duration_ = duration_sec;
}

// ==================== AudioEngine ====================

AudioEngine::AudioEngine() = default;
AudioEngine::~AudioEngine() { shutdown(); }

static void audio_callback_wrapper(ma_device* pDevice, void* pOutput,
                                    const void* pInput, ma_uint32 frameCount) {
    auto* engine = (AudioEngine*)pDevice->pUserData;
    if (engine) engine->mix_output((float*)pOutput, (int)frameCount);
}

bool AudioEngine::init(const AudioConfig& config) {
    config_ = config;
    
    // Initialize miniaudio engine
    ma_engine_config engine_config = ma_engine_config_init();
    engine_config.listenerCount = 1;
    engine_config.channels = 2;  // Stereo
    
    auto* engine = new ma_engine;
    ma_result result = ma_engine_init(&engine_config, engine);
    
    if (result != MA_SUCCESS) {
        std::cerr << "[Audio] Failed to initialize miniaudio: " << result << std::endl;
        delete engine;
        return false;
    }
    
    engine_ = engine;
    initialized_ = true;
    std::cout << "[Audio] Engine initialized (" << config.sample_rate << " Hz)" << std::endl;
    return true;
}

void AudioEngine::shutdown() {
    if (engine_) {
        auto* engine = (ma_engine*)engine_;
        ma_engine_uninit(engine);
        delete engine;
        engine_ = nullptr;
    }
    active_sources_.clear();
    initialized_ = false;
}

int AudioEngine::play_collision(const dvec3& position, const dvec3& velocity,
                                 double impulse, double mass_a, double mass_b,
                                 double restitution) {
    if (!initialized_) return -1;
    
    auto sound = std::make_shared<CollisionSound>();
    sound->generate(impulse, mass_a, mass_b, restitution, 0, 1); // metal → wood
    
    return play_sound(position, sound);
}

int AudioEngine::play_sound(const dvec3& position, std::shared_ptr<CollisionSound> sound,
                             float volume) {
    if (!initialized_ || !sound) return -1;
    
    AudioSource source;
    source.position = position;
    source.sound = sound;
    source.volume = volume * config_.sfx_volume * config_.master_volume;
    source.source_id = next_source_id_++;
    source.playing = true;
    source.duration = sound->duration();
    
    active_sources_.push_back(source);
    
    return source.source_id;
}

void AudioEngine::stop_source(int source_id) {
    for (auto& s : active_sources_) {
        if (s.source_id == source_id) {
            s.playing = false;
            break;
        }
    }
}

void AudioEngine::stop_all() {
    for (auto& s : active_sources_) s.playing = false;
}

float AudioEngine::calculate_distance_atten(const dvec3& source_pos, 
                                             const dvec3& listener_pos) {
    double dist = glm::length(source_pos - listener_pos);
    if (dist < 0.5) return 1.0f;
    return std::clamp(1.0f / (float)(dist * dist), 0.0f, 1.0f);
}

float AudioEngine::calculate_doppler(const dvec3& source_vel, const dvec3& listener_vel,
                                      const dvec3& source_to_listener) {
    float speed_of_sound = 343.0f;
    dvec3 dir = glm::normalize(source_to_listener);
    float v_s = glm::dot(source_vel, dir);
    float v_l = glm::dot(listener_vel, dir);
    return (speed_of_sound + v_l) / (speed_of_sound - v_s);
}

void AudioEngine::update(const dvec3& listener_pos, const dvec3& listener_vel,
                          const dvec3& listener_forward, const dvec3& listener_up,
                          double dt) {
    if (!initialized_) return;
    
    auto* engine = (ma_engine*)engine_;
    
    // Update listener
    ma_engine_listener_set_position(engine, 0,
        (float)listener_pos.x, (float)listener_pos.y, (float)listener_pos.z);
    ma_engine_listener_set_direction(engine, 0,
        (float)listener_forward.x, (float)listener_forward.y, (float)listener_forward.z);
    
    // Update active sources and remove finished ones
    active_sources_.erase(
        std::remove_if(active_sources_.begin(), active_sources_.end(),
            [&](AudioSource& s) {
                if (!s.playing) return true;
                s.play_time += (float)dt;
                if (s.play_time >= s.duration) return true;
                
                // Update spatial properties
                s.distance_atten = calculate_distance_atten(s.position, listener_pos);
                s.volume = config_.sfx_volume * config_.master_volume * s.distance_atten;
                
                return false;
            }),
        active_sources_.end());
}

void AudioEngine::mix_output(float* output, int frame_count) {
    // Clear output buffer
    memset(output, 0, frame_count * 2 * sizeof(float));
    
    for (const auto& source : active_sources_) {
        if (!source.playing || !source.sound) continue;
        
        const auto& buffer = source.sound->buffer();
        int sample_offset = (int)(source.play_time * config_.sample_rate);
        
        for (int i = 0; i < frame_count; i++) {
            int buf_idx = sample_offset + i;
            if (buf_idx >= (int)buffer.size()) break;
            
            float sample = buffer[buf_idx] * source.volume;
            
            // Simple panning based on position
            float pan = source.pan * config_.spatial_blend;
            output[i * 2 + 0] += sample * (0.5f - pan * 0.5f);  // Left
            output[i * 2 + 1] += sample * (0.5f + pan * 0.5f);  // Right
        }
    }
}

void AudioEngine::set_listener_pose(const dvec3& pos, const dvec3& forward,
                                     const dvec3& up, const dvec3& vel) {
    (void)up;
    if (!initialized_) return;
    auto* engine = (ma_engine*)engine_;
    ma_engine_listener_set_position(engine, 0,
        (float)pos.x, (float)pos.y, (float)pos.z);
    ma_engine_listener_set_direction(engine, 0,
        (float)forward.x, (float)forward.y, (float)forward.z);
}

} // namespace hlp
