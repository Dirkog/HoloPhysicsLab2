#pragma once

// ============================================================
// HoloPhysics Lab 2 — AR Occlusion & Depth Compositing
// 
// Решает проблему: рука пользователя всегда позади 3D объектов
// Решение: запись маски руки в depth buffer и
// шейдерный chroma key с прозрачностью
// ============================================================

#include "Renderer.h"
#include "../cv/HandTracker.h"
#include "../input/VirtualHand.h"
#include <vector>
#include <array>
#include <memory>

namespace hlp {

class OcclusionRenderer {
public:
    OcclusionRenderer(Renderer& renderer);
    ~OcclusionRenderer();

    // === Initialization ===
    void init(int width, int height);
    void shutdown();
    void resize(int width, int height);

    // === Occlusion rendering ===
    // Must be called BEFORE rendering physics objects
    void render_hand_occlusion(const VirtualHand& hand, 
                                const CameraIntrinsics& intrinsics);
    
    // === Chroma key compositing ===
    // Replaces green/blue screen with transparency
    void render_chroma_background();
    
    // === Foreground compositing ===
    // Renders hands ON TOP of physics (after physics render)
    void render_hand_foreground(const VirtualHand& hand,
                                 const CameraIntrinsics& intrinsics);

    // === Configuration ===
    void set_enabled(bool e) { enabled_ = e; }
    bool enabled() const { return enabled_; }
    
    void set_occlusion_strength(float s) { occ_strength_ = s; }
    void set_chroma_color(const glm::vec3& c) { chroma_color_ = c; }
    void set_chroma_threshold(float t) { chroma_threshold_ = t; }

    // === Hand silhouette generation ===
    // Creates a convex hull from landmarks for occlusion mask
    std::vector<dvec3> generate_hand_silhouette(const VirtualHand& hand) const;

private:
    Renderer& renderer_;
    bool enabled_ = true;
    float occ_strength_ = 1.0f;
    glm::vec3 chroma_color_{0.0f, 0.8f, 0.0f};
    float chroma_threshold_ = 0.3f;
    
    // FBO for occlusion pass
    GLuint occ_fbo_ = 0;
    GLuint occ_depth_tex_ = 0;
    GLuint occ_color_tex_ = 0;
    int fbo_width_ = 0, fbo_height_ = 0;
    
    // Shaders
    GLuint occ_shader_ = 0;
    GLuint compose_shader_ = 0;
    GLuint chroma_shader_ = 0;
    
    // Geometry
    GLuint occ_vao_ = 0;
    GLuint occ_vbo_ = 0;
    GLuint quad_vao_ = 0;
    
    // Silhouette generation
    static constexpr int HAND_SEGMENTS = 8;  // Points around each landmark
    
    // Helper: convert physics coords → render coords
    dvec3 to_render(const dvec3& v) const { return dvec3(v.x, v.z, v.y); }
    
    // Init helpers
    void init_fbo(int width, int height);
    void init_geometry();
    void init_occlusion_shaders();
    
    // Shader sources
    static const char* occ_vertex_src();
    static const char* occ_fragment_src();
    static const char* compose_vertex_src();
    static const char* compose_fragment_src();
    static const char* chroma_vertex_src();
    static const char* chroma_fragment_src();
};

} // namespace hlp
