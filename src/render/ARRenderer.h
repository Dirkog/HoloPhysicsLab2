#pragma once

// ============================================================
// HoloPhysics Lab 2 — AR Renderer
// Наложение видео с камеры, рендеринг скелета рук,
// визуализация точек взаимодействия
// ============================================================

#include "Renderer.h"
#include "../cv/CameraCapture.h"
#include "../input/VirtualHand.h"
#include "../interaction/HandInteraction.h"
#include <memory>

namespace hlp {

class ARRenderer {
public:
    ARRenderer(Renderer& renderer);
    ~ARRenderer();

    // === Initialization ===
    void init();
    void shutdown();

    // === Video Background ===
    void update_background_texture(const CameraFrame& frame);
    void render_background();
    void set_background_visible(bool v) { show_background_ = v; }
    bool background_visible() const { return show_background_; }

    // === Hand Visualization ===
    void render_hand_skeleton(const VirtualHand& hand, bool right_hand);
    void render_interaction_point(const dvec3& point, float radius,
                                   const glm::vec3& color);
    void render_ray(const dvec3& origin, const dvec3& end);
    void render_force_field(const dvec3& center, float radius);

    // === Debug Visualization ===
    void render_debug_info(const HandInteractionSystem::DebugInfo& debug);

    // === AR Settings ===
    void set_mirror_mode(bool m) { mirror_mode_ = m; }
    bool mirror_mode() const { return mirror_mode_; }
    
    void set_chroma_key_enabled(bool e) { chroma_key_ = e; }
    void set_chroma_key_color(const glm::vec3& c) { chroma_key_color_ = c; }
    
    void set_hand_opacity(float o) { hand_opacity_ = o; }
    float hand_opacity() const { return hand_opacity_; }

private:
    Renderer& renderer_;
    
    // State
    bool initialized_ = false;
    bool show_background_ = true;
    bool mirror_mode_ = true;
    bool chroma_key_ = false;
    glm::vec3 chroma_key_color_{0, 0.8f, 0};
    float hand_opacity_ = 0.85f;

    // === Video Background ===
    GLuint bg_vao_ = 0;
    GLuint bg_vbo_ = 0;
    GLuint bg_texture_ = 0;
    GLuint bg_shader_ = 0;
    int bg_width_ = 0, bg_height_ = 0;

    // === Hand Skeleton ===
    GLuint hand_shader_ = 0;
    GLuint hand_vao_ = 0;
    GLuint hand_vbo_ = 0;
    
    // === Interaction Point ===
    GLuint point_shader_ = 0;
    GLuint point_vao_ = 0;

    // === Ray ===
    GLuint ray_shader_ = 0;
    GLuint ray_vao_ = 0;

    // Skeletal bone connections
    static constexpr int BONES[][2] = {
        {0,1}, {1,2}, {2,3}, {3,4},           // Thumb
        {0,5}, {5,6}, {6,7}, {7,8},           // Index
        {0,9}, {9,10}, {10,11}, {11,12},      // Middle
        {0,13}, {13,14}, {14,15}, {15,16},    // Ring
        {0,17}, {17,18}, {18,19}, {19,20},    // Pinky
        {5,9}, {9,13}, {13,17},               // Palm
    };
    static constexpr int BONE_COUNT = 24;

    // === Internal helpers ===
    void init_background_geometry();
    void init_hand_geometry();
    void init_shaders();
    
    dvec3 world_to_render(const dvec3& v) const;

    // === Shader sources ===
    static const char* bg_vs();
    static const char* bg_fs();
    static const char* hand_vs();
    static const char* hand_fs();
    static const char* point_vs();
    static const char* point_fs();
};

} // namespace hlp
