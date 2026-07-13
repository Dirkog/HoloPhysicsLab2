#include "ARRenderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

namespace hlp {

ARRenderer::ARRenderer(Renderer& renderer) : renderer_(renderer) {}

ARRenderer::~ARRenderer() { shutdown(); }

void ARRenderer::init() {
    init_background_geometry();
    init_hand_geometry();
    init_shaders();
    initialized_ = true;
}

void ARRenderer::shutdown() {
    if (bg_vao_) glDeleteVertexArrays(1, &bg_vao_);
    if (bg_vbo_) glDeleteBuffers(1, &bg_vbo_);
    if (bg_texture_) glDeleteTextures(1, &bg_texture_);
    if (bg_shader_) glDeleteProgram(bg_shader_);
    if (hand_shader_) glDeleteProgram(hand_shader_);
    if (point_shader_) glDeleteProgram(point_shader_);
    if (ray_shader_) glDeleteProgram(ray_shader_);
}

void ARRenderer::init_background_geometry() {
    // Fullscreen quad in NDC
    float quad[] = {
        // pos        uv
        -1, -1,       0, 1,
         1, -1,       1, 1,
        -1,  1,       0, 0,
         1,  1,       1, 0,
    };
    
    glGenVertexArrays(1, &bg_vao_);
    glBindVertexArray(bg_vao_);
    
    glGenBuffers(1, &bg_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, bg_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    // Texture
    glGenTextures(1, &bg_texture_);
    glBindTexture(GL_TEXTURE_2D, bg_texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void ARRenderer::init_hand_geometry() {
    // Simple line and point rendering VAO
    glGenVertexArrays(1, &hand_vao_);
    glBindVertexArray(hand_vao_);
    
    glGenBuffers(1, &hand_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo_);
    glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Point VAO (interaction point as small sphere)
    glGenVertexArrays(1, &point_vao_);
    glBindVertexArray(point_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo_); // Reuse same VBO
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Ray VAO
    glGenVertexArrays(1, &ray_vao_);
    glBindVertexArray(ray_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void ARRenderer::init_shaders() {
    bg_shader_ = renderer_.compile_shader(bg_vs(), bg_fs());
    hand_shader_ = renderer_.compile_shader(hand_vs(), hand_fs());
    point_shader_ = renderer_.compile_shader(hand_vs(), hand_fs());
    ray_shader_ = renderer_.compile_shader(hand_vs(), hand_fs());
}

// ==================== Video Background ====================

void ARRenderer::update_background_texture(const CameraFrame& frame) {
    if (frame.rgb.empty()) return;
    
    bg_width_ = frame.rgb.cols;
    bg_height_ = frame.rgb.rows;
    
    glBindTexture(GL_TEXTURE_2D, bg_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, bg_width_, bg_height_,
                 0, GL_RGB, GL_UNSIGNED_BYTE, frame.rgb.data);
}

void ARRenderer::render_background() {
    if (!show_background_ || bg_width_ == 0) return;
    
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    
    glUseProgram(bg_shader_);
    glUniform1i(glGetUniformLocation(bg_shader_, "u_texture"), 0);
    glUniform1i(glGetUniformLocation(bg_shader_, "u_mirror"), mirror_mode_ ? 1 : 0);
    glUniform1i(glGetUniformLocation(bg_shader_, "u_chroma"), chroma_key_ ? 1 : 0);
    glUniform3fv(glGetUniformLocation(bg_shader_, "u_chroma_color"), 1, &chroma_key_color_[0]);
    
    glBindTexture(GL_TEXTURE_2D, bg_texture_);
    glBindVertexArray(bg_vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

// ==================== Hand Skeleton ====================

dvec3 ARRenderer::world_to_render(const dvec3& v) const {
    // Physics: +X right, +Y up, +Z forward
    // OpenGL: +X right, +Y up, -Z forward → convert Z
    return dvec3(v.x, v.y, v.z);
}

void ARRenderer::render_hand_skeleton(const VirtualHand& hand, bool right_hand) {
    if (!hand.is_active()) return;
    
    glm::mat4 vp = renderer_.proj_matrix() * renderer_.view_matrix();
    
    // === Draw Bones (Lines) ===
    glUseProgram(hand_shader_);
    glUniformMatrix4fv(glGetUniformLocation(hand_shader_, "u_viewProj"), 
                       1, GL_FALSE, &vp[0][0]);
    glUniform3f(glGetUniformLocation(hand_shader_, "u_color"), 
                right_hand ? 0.0f : 1.0f, 1.0f, right_hand ? 1.0f : 0.0f);
    glUniform1f(glGetUniformLocation(hand_shader_, "u_opacity"), hand_opacity_);
    
    // Upload bone vertices
    std::vector<float> line_verts;
    auto& world = hand.world_landmarks();
    
    for (int i = 0; i < BONE_COUNT; i++) {
        int a = BONES[i][0], b = BONES[i][1];
        dvec3 pa = world_to_render(world[a]);
        dvec3 pb = world_to_render(world[b]);
        
        line_verts.push_back((float)pa.x);
        line_verts.push_back((float)pa.y);
        line_verts.push_back((float)pa.z);
        line_verts.push_back((float)pb.x);
        line_verts.push_back((float)pb.y);
        line_verts.push_back((float)pb.z);
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo_);
    glBufferData(GL_ARRAY_BUFFER, line_verts.size() * sizeof(float), 
                 line_verts.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(hand_vao_);
    glLineWidth(3.0f);
    glDrawArrays(GL_LINES, 0, BONE_COUNT * 2);
    glLineWidth(1.0f);
    
    // === Draw Joints (Points) ===
    glUniform3f(glGetUniformLocation(hand_shader_, "u_color"), 
                0.2f, 1.0f, 0.3f);
    glUniform1f(glGetUniformLocation(hand_shader_, "u_opacity"), 0.9f);
    
    std::vector<float> joint_verts;
    for (int i = 0; i < HAND_LANDMARK_COUNT; i++) {
        dvec3 p = world_to_render(world[i]);
        joint_verts.push_back((float)p.x);
        joint_verts.push_back((float)p.y);
        joint_verts.push_back((float)p.z);
    }
    
    glBufferData(GL_ARRAY_BUFFER, joint_verts.size() * sizeof(float),
                 joint_verts.data(), GL_DYNAMIC_DRAW);
    
    glPointSize(6.0f);
    glDrawArrays(GL_POINTS, 0, HAND_LANDMARK_COUNT);
    glPointSize(1.0f);
}

void ARRenderer::render_interaction_point(const dvec3& point, float radius,
                                            const glm::vec3& color) {
    if (!initialized_) return;
    
    dvec3 rp = world_to_render(point);
    
    // Draw a glowing cross-hair at the interaction point
    glm::mat4 vp = renderer_.proj_matrix() * renderer_.view_matrix();
    
    glUseProgram(point_shader_);
    glUniformMatrix4fv(glGetUniformLocation(point_shader_, "u_viewProj"),
                       1, GL_FALSE, &vp[0][0]);
    glUniform3fv(glGetUniformLocation(point_shader_, "u_color"), 1, &color[0]);
    glUniform1f(glGetUniformLocation(point_shader_, "u_opacity"), 0.9f);
    glUniform1f(glGetUniformLocation(point_shader_, "u_radius"), radius);
    
    // Small cross-hair
    std::vector<float> cross = {
        (float)(rp.x - radius), (float)rp.y, (float)rp.z,
        (float)(rp.x + radius), (float)rp.y, (float)rp.z,
        (float)rp.x, (float)(rp.y - radius), (float)rp.z,
        (float)rp.x, (float)(rp.y + radius), (float)rp.z,
    };
    
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo_);
    glBufferData(GL_ARRAY_BUFFER, cross.size() * sizeof(float),
                 cross.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(ray_vao_);
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, 4);
    glLineWidth(1.0f);
}

void ARRenderer::render_ray(const dvec3& origin, const dvec3& end) {
    dvec3 ro = world_to_render(origin);
    dvec3 re = world_to_render(end);
    
    glm::mat4 vp = renderer_.proj_matrix() * renderer_.view_matrix();
    
    glUseProgram(ray_shader_);
    glUniformMatrix4fv(glGetUniformLocation(ray_shader_, "u_viewProj"),
                       1, GL_FALSE, &vp[0][0]);
    glUniform3f(glGetUniformLocation(ray_shader_, "u_color"), 1.0f, 0.3f, 0.1f);
    glUniform1f(glGetUniformLocation(ray_shader_, "u_opacity"), 0.5f);
    
    std::vector<float> ray_verts = {
        (float)ro.x, (float)ro.y, (float)ro.z,
        (float)re.x, (float)re.y, (float)re.z,
    };
    
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo_);
    glBufferData(GL_ARRAY_BUFFER, ray_verts.size() * sizeof(float),
                 ray_verts.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(ray_vao_);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(1, 0xCCCC);
    glDrawArrays(GL_LINES, 0, 2);
    glDisable(GL_LINE_STIPPLE);
}

void ARRenderer::render_force_field(const dvec3& center, float radius) {
    dvec3 rc = world_to_render(center);
    
    glm::mat4 vp = renderer_.proj_matrix() * renderer_.view_matrix();
    
    glUseProgram(point_shader_);
    glUniformMatrix4fv(glGetUniformLocation(point_shader_, "u_viewProj"),
                       1, GL_FALSE, &vp[0][0]);
    glUniform3f(glGetUniformLocation(point_shader_, "u_color"), 0.5f, 0.3f, 1.0f);
    glUniform1f(glGetUniformLocation(point_shader_, "u_opacity"), 0.15f);
    glUniform1f(glGetUniformLocation(point_shader_, "u_radius"), 0.1f);
    
    // Draw a ring (simplified as a line loop)
    std::vector<float> ring;
    int segments = 32;
    for (int i = 0; i <= segments; i++) {
        float angle = (float)i / segments * 2.0f * (float)M_PI;
        ring.push_back((float)(rc.x + radius * std::cos(angle)));
        ring.push_back((float)(rc.y + radius * std::sin(angle) * 0.5f)); // Flatten
        ring.push_back((float)rc.z);
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo_);
    glBufferData(GL_ARRAY_BUFFER, ring.size() * sizeof(float),
                 ring.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(ray_vao_);
    glDrawArrays(GL_LINE_STRIP, 0, segments + 1);
}

void ARRenderer::render_debug_info(const HandInteractionSystem::DebugInfo& debug) {
    if (debug.show_ray) {
        render_ray(debug.interaction_point, debug.ray_end);
    }
    
    if (debug.show_force_field) {
        render_force_field(debug.interaction_point, 0.8f);
    }
    
    render_interaction_point(debug.interaction_point, 0.04f,
                              glm::vec3(0.0f, 1.0f, 0.8f));
}

// ==================== Shaders ====================

const char* ARRenderer::bg_vs() {
    return R"(#version 460 core
    layout(location = 0) in vec2 a_pos;
    layout(location = 1) in vec2 a_uv;
    uniform int u_mirror;
    out vec2 v_uv;
    void main() {
        v_uv = vec2(u_mirror == 1 ? 1.0 - a_uv.x : a_uv.x, a_uv.y);
        gl_Position = vec4(a_pos, 0.0, 1.0);
    })";
}

const char* ARRenderer::bg_fs() {
    return R"(#version 460 core
    in vec2 v_uv;
    uniform sampler2D u_texture;
    uniform int u_chroma;
    uniform vec3 u_chroma_color;
    out vec4 fragColor;
    void main() {
        vec4 tex = texture(u_texture, v_uv);
        if (u_chroma == 1) {
            float diff = length(tex.rgb - u_chroma_color);
            if (diff < 0.3) discard;
        }
        fragColor = tex;
    })";
}

const char* ARRenderer::hand_vs() {
    return R"(#version 460 core
    layout(location = 0) in vec3 a_pos;
    uniform mat4 u_viewProj;
    uniform float u_radius;
    void main() {
        gl_Position = u_viewProj * vec4(a_pos, 1.0);
        if (u_radius > 0.0) {
            gl_PointSize = max(1.0, u_radius * 100.0 / gl_Position.w);
        }
    })";
}

const char* ARRenderer::hand_fs() {
    return R"(#version 460 core
    uniform vec3 u_color;
    uniform float u_opacity;
    out vec4 fragColor;
    void main() {
        fragColor = vec4(u_color, u_opacity);
    })";
}

const char* ARRenderer::point_vs() { return hand_vs(); }
const char* ARRenderer::point_fs() { return hand_fs(); }

} // namespace hlp
