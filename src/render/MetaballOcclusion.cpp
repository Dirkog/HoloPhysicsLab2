#include "MetaballOcclusion.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <cmath>

namespace hlp {

// ==================== MetaballOcclusion ====================

MetaballOcclusion::MetaballOcclusion(Renderer& renderer) : renderer_(renderer) {}
MetaballOcclusion::~MetaballOcclusion() { shutdown(); }

void MetaballOcclusion::init(int width, int height) {
    init_fbo(width, height);
    init_quad();
    init_shader();
    std::cout << "[Metaball] Initialized " << width << "×" << height << std::endl;
}

void MetaballOcclusion::shutdown() {
    if (occ_fbo_) glDeleteFramebuffers(1, &occ_fbo_);
    if (occ_depth_) glDeleteTextures(1, &occ_depth_);
    if (quad_vao_) glDeleteVertexArrays(1, &quad_vao_);
    if (occ_shader_) glDeleteProgram(occ_shader_);
}

void MetaballOcclusion::resize(int width, int height) {
    shutdown();
    init(width, height);
}

void MetaballOcclusion::init_fbo(int w, int h) {
    fbo_w_ = w;
    fbo_h_ = h;
    
    glGenFramebuffers(1, &occ_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, occ_fbo_);
    
    // High-precision depth texture
    glGenTextures(1, &occ_depth_);
    glBindTexture(GL_TEXTURE_2D, occ_depth_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, w, h,
                 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, occ_depth_, 0);
    
    // No color buffer needed (depth-only)
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[Metaball] FBO error: " << status << std::endl;
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MetaballOcclusion::init_quad() {
    float verts[] = {
        -1, -1, 0, 0,   1, -1, 1, 0,
        -1,  1, 0, 1,   1,  1, 1, 1,
    };
    
    glGenVertexArrays(1, &quad_vao_);
    glBindVertexArray(quad_vao_);
    
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)(2*sizeof(float)));
    glEnableVertexAttribArray(1);
}

void MetaballOcclusion::init_shader() {
    occ_shader_ = renderer_.compile_shader(vertex_src(), fragment_src());
    
    u_viewProj_ = glGetUniformLocation(occ_shader_, "u_viewProj");
    u_joints_ = glGetUniformLocation(occ_shader_, "u_joints");
    u_radii_ = glGetUniformLocation(occ_shader_, "u_radii");
    u_num_joints_ = glGetUniformLocation(occ_shader_, "u_num_joints");
    u_smooth_k_ = glGetUniformLocation(occ_shader_, "u_smooth_k");
    u_visible_ = glGetUniformLocation(occ_shader_, "u_visible");
    u_viewDir_ = glGetUniformLocation(occ_shader_, "u_viewDir");
}

MetaballOcclusion::HandJointsGL MetaballOcclusion::build_joints(const VirtualHand& hand) const {
    HandJointsGL data;
    auto& world = hand.world_landmarks();
    
    for (int i = 0; i < 21; i++) {
        dvec3 p = to_render(world[i]);
        data.pos[i * 3 + 0] = (float)p.x;
        data.pos[i * 3 + 1] = (float)p.y;
        data.pos[i * 3 + 2] = (float)p.z;
        
        // Per-joint radius: fingertips smaller, palm larger
        if (i >= 4 && i <= 20 && (i % 4 == 0)) {
            // Fingertips: smaller
            data.radius[i] = (float)joint_radius_ * 0.7f;
        } else if (i == 0) {
            // Wrist: larger
            data.radius[i] = (float)joint_radius_ * 1.5f;
        } else {
            data.radius[i] = (float)joint_radius_;
        }
    }
    
    return data;
}

void MetaballOcclusion::render_occlusion(const VirtualHand& hand) {
    if (!enabled_ || !hand.is_active()) return;
    
    // === Pass 1: Render metaball depth to FBO ===
    glBindFramebuffer(GL_FRAMEBUFFER, occ_fbo_);
    glViewport(0, 0, fbo_w_, fbo_h_);
    glClear(GL_DEPTH_BUFFER_BIT);
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glDepthMask(GL_TRUE);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE); // No color writes
    
    glUseProgram(occ_shader_);
    
    // Camera matrices
    glm::mat4 vp = renderer_.proj_matrix() * renderer_.view_matrix();
    glUniformMatrix4fv(u_viewProj_, 1, GL_FALSE, &vp[0][0]);
    
    // View direction for billboarding
    glm::mat4 view = renderer_.view_matrix();
    glm::vec3 view_dir(-view[0][2], -view[1][2], -view[2][2]);
    glUniform3fv(u_viewDir_, 1, &view_dir[0]);
    
    // Hand joints
    auto joints = build_joints(hand);
    glUniform3fv(u_joints_, 21, joints.pos);
    glUniform1fv(u_radii_, 21, joints.radius);
    glUniform1i(u_num_joints_, 21);
    glUniform1f(u_smooth_k_, (float)smooth_k_);
    glUniform1i(u_visible_, visible_ ? 1 : 0);
    
    // Render fullscreen quad with raycasting shader
    glBindVertexArray(quad_vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    // === Pass 2: Blend occlusion into main depth buffer ===
    // (In production: bind occ_depth_ as texture and do full-screen compose)
    glBindTexture(GL_TEXTURE_2D, occ_depth_);
    
    // Use glBlitFramebuffer to copy depth
    glBindFramebuffer(GL_READ_FRAMEBUFFER, occ_fbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, fbo_w_, fbo_h_, 0, 0, fbo_w_, fbo_h_,
                      GL_DEPTH_BUFFER_BIT, GL_NEAREST);
}

// ==================== GLSL Shaders ====================

const char* MetaballOcclusion::vertex_src() {
    return R"(#version 460 core
    layout(location = 0) in vec2 a_pos;
    layout(location = 1) in vec2 a_uv;
    out vec2 v_uv;
    void main() {
        v_uv = a_uv;
        gl_Position = vec4(a_pos, 0.0, 1.0);
    })";
}

const char* MetaballOcclusion::fragment_src() {
    return R"(#version 460 core
    in vec2 v_uv;
    
    uniform mat4 u_viewProj;
    uniform vec3 u_joints[21];
    uniform float u_radii[21];
    uniform int u_num_joints;
    uniform float u_smooth_k;
    uniform bool u_visible;
    uniform vec3 u_viewDir;
    
    // Inverse of VP matrix for ray construction
    uniform mat4 u_invVP;
    
    out float fragDepth;
    
    // Smooth minimum for organic blending
    float smin(float a, float b, float k) {
        float h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
        return mix(b, a, h) - k * h * (1.0 - h);
    }
    
    // Distance from point to line segment (for bones)
    float dist_to_segment(vec3 p, vec3 a, vec3 b, float ra, float rb) {
        vec3 ab = b - a;
        float t = clamp(dot(p - a, ab) / dot(ab, ab), 0.0, 1.0);
        vec3 closest = a + ab * t;
        float r = mix(ra, rb, t);
        return length(p - closest) - r;
    }
    
    void main() {
        // Construct ray from camera through pixel
        vec4 ndc = vec4(v_uv * 2.0 - 1.0, 1.0, 1.0);
        vec4 eye_ray = u_invVP * ndc;
        vec3 ray_origin = eye_ray.xyz / eye_ray.w;
        vec3 ray_dir = normalize(ray_origin);  // In view space
        
        // Simplified: use joint positions directly
        // For each pixel, compute SDF value from metaballs
        
        vec3 pixel_world = ray_origin;  // Approximate
        
        float sdf = 1e10;
        
        // Joints as spheres
        for (int i = 0; i < u_num_joints; i++) {
            float d = length(pixel_world - u_joints[i]) - u_radii[i];
            sdf = smin(sdf, d, u_smooth_k);
        }
        
        // Bones as capsules (smooth cylinders between joints)
        // Hand bone connections (simplified subset)
        int bones[][2] = int[][2](
            {0,1}, {1,2}, {2,3}, {3,4},       // Thumb
            {0,5}, {5,6}, {6,7}, {7,8},       // Index
            {0,9}, {9,10}, {10,11}, {11,12},  // Middle
            {0,13}, {13,14}, {14,15}, {15,16},// Ring
            {0,17}, {17,18}, {18,19}, {19,20},// Pinky
            {5,9}, {9,13}, {13,17}            // Palm
        );
        
        for (int b = 0; b < 23; b++) {
            int i = bones[b][0];
            int j = bones[b][1];
            float d = dist_to_segment(pixel_world, u_joints[i], u_joints[j], 
                                      u_radii[i] * 0.6, u_radii[j] * 0.6);
            sdf = smin(sdf, d, u_smooth_k);
        }
        
        // If inside hand (SDF < 0), write depth to occlude
        if (sdf < 0.0) {
            // Write a fixed depth (hand is between near and far plane)
            gl_FragDepth = 0.5;  // Closer than most objects
        } else {
            discard;  // Don't occlude
        }
        
        fragDepth = gl_FragDepth;
    })";
}

// ==================== ProximityFeedback ====================

std::vector<ProximityFeedback::GlowInfo> ProximityFeedback::compute_glow(
    const std::vector<Body>& bodies,
    const VirtualHand& hand,
    const VirtualHandBody& virtual_hand,
    double time) {
    
    std::vector<GlowInfo> result;
    dvec3 hand_pos = virtual_hand.state.position;
    
    for (const auto& b : bodies) {
        if (b.is_static()) continue;
        
        double dist = glm::length(b.state.position - hand_pos);
        double threshold = b.props.radius + glow_radius + virtual_hand.radius;
        
        if (dist < threshold) {
            GlowInfo gi;
            gi.body_id = b.id;
            gi.proximity = (float)std::clamp(1.0 - dist / threshold, 0.0, 1.0);
            gi.glow_color = dvec3(glow_color.r, glow_color.g, glow_color.b);
            gi.pulse_phase = (float)(std::sin(time * pulse_speed + b.id * 2.0) * 0.5 + 0.5);
            result.push_back(gi);
        }
    }
    
    return result;
}

void ProximityFeedback::apply_glow(std::vector<Body>& bodies, 
                                    const std::vector<GlowInfo>& glow) {
    for (auto& b : bodies) {
        // Reset all bodies first
        b.visual.emissive_intensity = 0.15f;
        
        // Apply glow
        for (const auto& g : glow) {
            if (b.id == g.body_id) {
                float intensity = g.proximity * max_emissive * (0.5f + 0.5f * g.pulse_phase);
                b.visual.emissive_intensity = intensity;
                
                // Tint color toward glow color
                glm::vec3 base = b.visual.color;
                float blend = g.proximity * 0.3f;
                b.visual.color = base * (1.0f - blend) + glow_color * blend;
            }
        }
    }
}

} // namespace hlp
