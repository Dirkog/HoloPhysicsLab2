#pragma once

// ============================================================
// HoloPhysics Lab 2 — GPU-Optimized Metaball Occlusion
//
// Решает проблему производительности Raymarching:
//   1. Screen-Space Scissor Rect (только область руки)
//   2. Hierarchical SDF (6 кластеров → 44 примитива)
//   3. LOD: дальние пиксели = грубая модель
// ============================================================

#include "Renderer.h"
#include "../input/VirtualHand.h"
#include <array>
#include <vector>

namespace hlp {

struct HandBounds {
    // Screen-space AABB (in pixels)
    int min_x, min_y, max_x, max_y;
    bool valid = false;
    
    // World-space bounding spheres for hierarchy
    struct Cluster {
        dvec3 center;
        double radius;
        int start_idx;  // First joint index in cluster
        int end_idx;    // Last joint index in cluster
    };
    std::array<Cluster, 6> clusters;  // Palm + 5 fingers
    
    // Joint positions for full SDF
    std::array<dvec3, 21> joints;
    std::array<double, 21> radii;
};

class OptimizedMetaball {
public:
    OptimizedMetaball(Renderer& renderer);
    ~OptimizedMetaball();

    void init(int width, int height);
    void shutdown();
    void resize(int width, int height);

    // === Main render (optimized) ===
    void render_occlusion(const VirtualHand& hand,
                          const glm::mat4& view_proj,
                          int viewport_w, int viewport_h);

    // === Configuration ===
    void set_enabled(bool e) { enabled_ = e; }
    bool enabled() const { return enabled_; }
    
    void set_quality(int q) { quality_ = q; }  // 0=low, 1=med, 2=high
    int quality() const { return quality_; }
    
    // Stats
    int last_pixel_count() const { return last_pixels_; }
    int last_primitives_tested() const { return last_primitives_; }

private:
    Renderer& renderer_;
    bool enabled_ = true;
    int quality_ = 1;
    
    // Performance tracking
    int last_pixels_ = 0;
    int last_primitives_ = 0;
    
    // FBO
    GLuint occ_fbo_ = 0;
    GLuint occ_depth_ = 0;
    int fbo_w_ = 0, fbo_h_ = 0;
    
    // Shader
    GLuint occ_shader_ = 0;
    GLuint compose_shader_ = 0;
    
    // Fullscreen quad
    GLuint quad_vao_ = 0;
    
    // === Bounding box computation ===
    HandBounds compute_hand_bounds(const VirtualHand& hand,
                                    const glm::mat4& view_proj,
                                    int viewport_w, int viewport_h);
    
    // === Hierarchical pruning ===
    // Returns which clusters are active for a given pixel ray
    int get_active_clusters(const dvec3& ray_origin, const dvec3& ray_dir,
                            const HandBounds& bounds);
    
    // === GPU data upload ===
    void upload_joints_to_shader(const HandBounds& bounds);
    void upload_clusters_to_shader(const HandBounds& bounds);
    
    // Init helpers
    void init_fbo(int w, int h);
    void init_quad();
    void init_shaders();
    
    // Shaders
    static const char* occ_vs();
    static const char* occ_fs();
    static const char* compose_vs();
    static const char* compose_fs();
    
    // Uniform locations
    GLint u_viewProj_;
    GLint u_joints_;
    GLint u_radii_;
    GLint u_num_joints_;
    GLint u_smooth_k_;
    GLint u_clusters_;
    GLint u_num_clusters_;
    GLint u_scissor_;
    GLint u_viewport_;
    GLint u_quality_;
};

} // namespace hlp
