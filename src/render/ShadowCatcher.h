#pragma once

// ============================================================
// HoloPhysics Lab 2 — AR Shadow Catcher
//
// Создаёт иллюзию, что виртуальные объекты отбрасывают
// тени на реальный стол/пол.
//
// Алгоритм:
//   1. Плоскость стола (калибруется один раз)
//   2. Shadow map от виртуального источника света
//   3. Композитинг: затемнение видеофона в области тени
//   4. Оценка направления света из видео (опционально)
// ============================================================

#include "Renderer.h"
#include "../core/Types.h"
#include <glm/glm.hpp>
#include <vector>

namespace hlp {

struct ShadowCatcherConfig {
    bool enabled = true;
    int shadow_map_size = 512;
    float shadow_intensity = 0.4f;    // [0,1] how dark the shadow is
    float shadow_blur = 2.0f;         // PCF blur radius
    float max_shadow_distance = 20.0f; // Max distance for shadow casting
    
    // Light estimation
    bool auto_light_direction = false; // Auto-estimate from video
    glm::vec3 light_direction{0.3f, 0.8f, 0.5f}; // Default light dir
    
    // Ground plane (calibrated)
    glm::vec3 ground_normal{0.0f, 1.0f, 0.0f};
    float ground_offset = 0.0f;  // Distance from origin along normal
};

class ShadowCatcher {
public:
    ShadowCatcher(Renderer& renderer);
    ~ShadowCatcher();

    // === Initialization ===
    void init(int shadow_map_size = 512);
    void shutdown();
    void resize(int shadow_map_size) { init(shadow_map_size); }

    // === Render shadow map (call before main render) ===
    void begin_shadow_pass();
    void end_shadow_pass();
    
    // === Render shadow onto background (call during compose) ===
    void render_shadow(const std::vector<Body>& bodies,
                       const glm::mat4& view_proj,
                       const glm::mat4& light_vp);
    
    // === Full pipeline ===
    void render_shadows(const std::vector<Body>& bodies,
                        const glm::mat4& view_proj);

    // === Light estimation from hand brightness ===
    void estimate_light_direction(const cv::Mat& camera_frame,
                                   const std::vector<dvec3>& hand_landmarks_2d);

    // === Ground plane calibration ===
    void calibrate_ground_plane(const dvec3& point_on_ground,
                                 const dvec3& normal = dvec3(0, 1, 0));
    
    // === Configuration ===
    void set_config(const ShadowCatcherConfig& cfg) { config_ = cfg; }
    ShadowCatcherConfig& config() { return config_; }
    const ShadowCatcherConfig& config() const { return config_; }
    
    // === Light VP matrix ===
    glm::mat4 light_view_matrix() const;
    glm::mat4 light_proj_matrix() const;
    glm::mat4 light_view_proj() const { return light_vp_; }

private:
    Renderer& renderer_;
    ShadowCatcherConfig config_;
    
    // Shadow map FBO
    GLuint shadow_fbo_ = 0;
    GLuint shadow_depth_tex_ = 0;
    int shadow_size_ = 512;
    
    // Compose shader for shadow overlay
    GLuint shadow_shader_ = 0;
    GLuint compose_vao_ = 0;
    
    // Light space matrices
    glm::mat4 light_vp_{1.0f};
    glm::vec3 light_dir_{0.3f, 0.8f, 0.5f};
    glm::vec3 light_target_{0.0f, 0.0f, 0.0f};
    
    // Shadow map FBO
    void init_fbo();
    void init_shader();
    void init_compose_quad();
    
    // Estimate light from image gradient
    glm::vec3 estimate_from_gradient(const cv::Mat& frame);
    
    // Shader sources
    static const char* shadow_vs();
    static const char* shadow_fs();
    static const char* compose_vs();
    static const char* compose_fs();
};

} // namespace hlp
