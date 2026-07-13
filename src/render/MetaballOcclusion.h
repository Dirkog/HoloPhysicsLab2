#pragma once

// ============================================================
// HoloPhysics Lab 2 — Metaball Occlusion Renderer
// 
// Решает проблему "сосисочной окклюзии" (sausage occlusion):
// Вместо цилиндров между суставами, использует SDF с smooth-min
// для создания органичной 3D маски руки.
//
// Все вычисления — в фрагментном шейдере, без CPU-геометрии.
// ============================================================

#include "Renderer.h"
#include "../input/VirtualHand.h"
#include <array>

namespace hlp {

class MetaballOcclusion {
public:
    MetaballOcclusion(Renderer& renderer);
    ~MetaballOcclusion();

    void init(int width, int height);
    void shutdown();
    void resize(int width, int height);

    // === Main render ===
    // Renders hand occlusion mask using metaball SDF
    void render_occlusion(const VirtualHand& hand);
    
    // === Configuration ===
    void set_enabled(bool e) { enabled_ = e; }
    bool enabled() const { return enabled_; }
    
    void set_visibility(bool v) { visible_ = v; }
    bool visibility() const { return visible_; }
    
    // Metaball parameters
    void set_smooth_k(double k) { smooth_k_ = k; }
    void set_joint_radius(double r) { joint_radius_ = r; }
    void set_bone_radius(double r) { bone_radius_ = r; }
    
    // === Utility: hand joint positions to shader uniform ===
    struct HandJointsGL {
        float pos[21 * 3];  // 21 joints × XYZ
        float radius[21];   // Per-joint radius
    };

private:
    Renderer& renderer_;
    bool enabled_ = true;
    bool visible_ = true;
    
    // Metaball params
    double smooth_k_ = 0.15;       // Smooth minimum blend factor
    double joint_radius_ = 0.04;   // 4cm per joint
    double bone_radius_ = 0.025;   // 2.5cm for bones
    
    // FBO for occlusion
    GLuint occ_fbo_ = 0;
    GLuint occ_depth_ = 0;
    int fbo_w_ = 0, fbo_h_ = 0;
    
    // Shader
    GLuint occ_shader_ = 0;
    
    // Fullscreen quad
    GLuint quad_vao_ = 0;
    
    // Init helpers
    void init_fbo(int w, int h);
    void init_quad();
    void init_shader();
    
    // Build joint data for shader
    HandJointsGL build_joints(const VirtualHand& hand) const;
    
    // Coordinate conversion
    dvec3 to_render(const dvec3& v) const { return dvec3(v.x, v.z, v.y); }
    
    // === GLSL Shaders ===
    static const char* vertex_src();
    static const char* fragment_src();
    
    // Shader uniform locations
    GLint u_viewProj_;
    GLint u_joints_;
    GLint u_radii_;
    GLint u_num_joints_;
    GLint u_smooth_k_;
    GLint u_visible_;
    GLint u_viewDir_;
};

// ==================== Screen-Space Hand Glow ====================

class ProximityFeedback {
public:
    ProximityFeedback() = default;
    
    struct GlowInfo {
        uint64_t body_id;
        float proximity;    // [0,1], 1 = touching
        dvec3 glow_color;
        float pulse_phase;
    };
    
    // Compute proximity glow for all bodies near hand
    std::vector<GlowInfo> compute_glow(
        const std::vector<Body>& bodies,
        const VirtualHand& hand,
        const VirtualHandBody& virtual_hand,
        double time);
    
    // Modify body emissive based on proximity
    void apply_glow(std::vector<Body>& bodies, const std::vector<GlowInfo>& glow);
    
    // Configuration
    double glow_radius = 0.15;      // Distance for glow to start
    double max_glow_distance = 0.3; // Distance for max glow
    glm::vec3 glow_color{0.0f, 1.0f, 0.8f};
    float pulse_speed = 2.0f;
    float max_emissive = 0.6f;
};

} // namespace hlp
