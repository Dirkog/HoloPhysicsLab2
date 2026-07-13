#include "OcclusionRenderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <iostream>

namespace hlp {

OcclusionRenderer::OcclusionRenderer(Renderer& renderer) : renderer_(renderer) {}
OcclusionRenderer::~OcclusionRenderer() { shutdown(); }

void OcclusionRenderer::init(int width, int height) {
    init_fbo(width, height);
    init_geometry();
    init_occlusion_shaders();
}

void OcclusionRenderer::shutdown() {
    if (occ_fbo_) glDeleteFramebuffers(1, &occ_fbo_);
    if (occ_depth_tex_) glDeleteTextures(1, &occ_depth_tex_);
    if (occ_color_tex_) glDeleteTextures(1, &occ_color_tex_);
    if (occ_vao_) glDeleteVertexArrays(1, &occ_vao_);
    if (occ_vbo_) glDeleteBuffers(1, &occ_vbo_);
    if (quad_vao_) glDeleteVertexArrays(1, &quad_vao_);
    if (occ_shader_) glDeleteProgram(occ_shader_);
    if (compose_shader_) glDeleteProgram(compose_shader_);
    if (chroma_shader_) glDeleteProgram(chroma_shader_);
}

void OcclusionRenderer::resize(int width, int height) {
    shutdown();
    init(width, height);
}

void OcclusionRenderer::init_fbo(int width, int height) {
    fbo_width_ = width;
    fbo_height_ = height;
    
    // Create FBO
    glGenFramebuffers(1, &occ_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, occ_fbo_);
    
    // Depth texture (for occlusion)
    glGenTextures(1, &occ_depth_tex_);
    glBindTexture(GL_TEXTURE_2D, occ_depth_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height,
                 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, 
                           GL_TEXTURE_2D, occ_depth_tex_, 0);
    
    // Color texture (for debug viewing)
    glGenTextures(1, &occ_color_tex_);
    glBindTexture(GL_TEXTURE_2D, occ_color_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, occ_color_tex_, 0);
    
    GLenum draw_buffers[] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, draw_buffers);
    
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[Occlusion] FBO not complete!" << std::endl;
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OcclusionRenderer::init_geometry() {
    // Silhouette VAO (will be updated dynamically)
    glGenVertexArrays(1, &occ_vao_);
    glBindVertexArray(occ_vao_);
    glGenBuffers(1, &occ_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, occ_vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Fullscreen quad for composing
    float quad[] = {
        -1, -1, 0, 0,   1, -1, 1, 0,
        -1,  1, 0, 1,   1,  1, 1, 1,
    };
    glGenVertexArrays(1, &quad_vao_);
    glBindVertexArray(quad_vao_);
    GLuint quad_vbo;
    glGenBuffers(1, &quad_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2*sizeof(float)));
    glEnableVertexAttribArray(1);
}

void OcclusionRenderer::init_occlusion_shaders() {
    occ_shader_ = renderer_.compile_shader(occ_vertex_src(), occ_fragment_src());
    compose_shader_ = renderer_.compile_shader(compose_vertex_src(), compose_fragment_src());
    chroma_shader_ = renderer_.compile_shader(chroma_vertex_src(), chroma_fragment_src());
}

// ==================== Silhouette Generation ====================

std::vector<dvec3> OcclusionRenderer::generate_hand_silhouette(const VirtualHand& hand) const {
    std::vector<dvec3> silhouette;
    if (!hand.is_active()) return silhouette;
    
    auto& world = hand.world_landmarks();
    
    // Create a convex hull-like shape from hand outline
    // Use fingertip points + wrist as key outline vertices
    const int outline_indices[] = {
        HAND_WRIST,
        HAND_THUMB_CMC, HAND_THUMB_MCP, HAND_THUMB_IP, HAND_THUMB_TIP,
        HAND_INDEX_TIP,
        HAND_MIDDLE_TIP,
        HAND_RING_TIP,
        HAND_PINKY_TIP,
        HAND_PINKY_MCP, HAND_PINKY_PIP,
        HAND_WRIST,  // Close the loop
    };
    
    // Expand each point into a small disc for volume
    for (int idx : outline_indices) {
        dvec3 center = world[idx];
        float radius = 0.02f;  // 2cm around each landmark
        
        // Add several points in a ring around each landmark
        for (int j = 0; j < 4; j++) {
            float angle = (float)j / 4 * 2.0f * (float)M_PI;
            dvec3 offset(
                radius * std::cos(angle),
                radius * std::sin(angle) * 0.5f,  // Flatten in Y
                0.01f * std::sin(angle * 2)
            );
            silhouette.push_back(to_render(center + offset));
        }
    }
    
    return silhouette;
}

// ==================== Occlusion Pass ====================

void OcclusionRenderer::render_hand_occlusion(const VirtualHand& hand,
                                               const CameraIntrinsics& intrinsics) {
    if (!enabled_ || !hand.is_active()) return;
    
    auto silhouette = generate_hand_silhouette(hand);
    if (silhouette.empty()) return;
    
    glm::mat4 vp = renderer_.proj_matrix() * renderer_.view_matrix();
    
    // Bind FBO and write ONLY to depth
    glBindFramebuffer(GL_FRAMEBUFFER, occ_fbo_);
    glViewport(0, 0, fbo_width_, fbo_height_);
    glClear(GL_DEPTH_BUFFER_BIT);
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);  // Always write depth (we ARE the hand)
    glDepthMask(GL_TRUE);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);  // Don't write color
    
    glUseProgram(occ_shader_);
    glUniformMatrix4fv(glGetUniformLocation(occ_shader_, "u_viewProj"),
                       1, GL_FALSE, &vp[0][0]);
    glUniform1f(glGetUniformLocation(occ_shader_, "u_depth"),
                hand.is_active() ? 0.5f : 1.0f);  // Hand is at 0.5 in depth
    
    // Upload silhouette mesh
    std::vector<float> verts;
    for (const auto& v : silhouette) {
        verts.push_back((float)v.x);
        verts.push_back((float)v.y);
        verts.push_back((float)v.z);
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, occ_vbo_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float),
                 verts.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(occ_vao_);
    glPointSize(8.0f);
    glDrawArrays(GL_POINTS, 0, (int)silhouette.size());
    
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    // Now copy the depth texture to the default FBO for occlusion
    // In a full implementation: blit or bind as texture
    glBindTexture(GL_TEXTURE_2D, occ_depth_tex_);
    
    // Set up the compose pass
    glUseProgram(compose_shader_);
    glUniform1i(glGetUniformLocation(compose_shader_, "u_depthTex"), 0);
    glUniform1f(glGetUniformLocation(compose_shader_, "u_near"), 0.1f);
    glUniform1f(glGetUniformLocation(compose_shader_, "u_far"), 100.0f);
    glUniform1f(glGetUniformLocation(compose_shader_, "u_handDepth"), 0.5f);
    glUniform1f(glGetUniformLocation(compose_shader_, "u_strength"), occ_strength_);
    
    // This would be applied as a full-screen pass before rendering physics
    // For simplicity: we render the occlusion as a billboard overlay
}

void OcclusionRenderer::render_chroma_background() {
    // Full-screen pass: replace chroma key color with transparency
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    
    glUseProgram(chroma_shader_);
    glUniform1i(glGetUniformLocation(chroma_shader_, "u_bgTex"), 0);
    glUniform3fv(glGetUniformLocation(chroma_shader_, "u_chromaColor"), 
                 1, &chroma_color_[0]);
    glUniform1f(glGetUniformLocation(chroma_shader_, "u_threshold"), chroma_threshold_);
    
    glBindVertexArray(quad_vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

void OcclusionRenderer::render_hand_foreground(const VirtualHand& hand,
                                                 const CameraIntrinsics& intrinsics) {
    if (!enabled_ || !hand.is_active()) return;
    // Render hand skeleton ON TOP of everything
    // (delegated to ARRenderer for now)
}

// ==================== Shaders ====================

const char* OcclusionRenderer::occ_vertex_src() {
    return R"(#version 460 core
    layout(location = 0) in vec3 a_pos;
    uniform mat4 u_viewProj;
    uniform float u_depth;
    void main() {
        gl_Position = u_viewProj * vec4(a_pos, 1.0);
        gl_PointSize = 16.0;
    })";
}

const char* OcclusionRenderer::occ_fragment_src() {
    return R"(#version 460 core
    uniform float u_depth;
    out vec4 fragColor;
    void main() {
        // Write a fixed depth value for the hand
        gl_FragDepth = u_depth;
        fragColor = vec4(0, 0, 0, 0);  // Invisible
    })";
}

const char* OcclusionRenderer::compose_vertex_src() {
    return R"(#version 460 core
    layout(location = 0) in vec2 a_pos;
    layout(location = 1) in vec2 a_uv;
    out vec2 v_uv;
    void main() {
        v_uv = a_uv;
        gl_Position = vec4(a_pos, 0.0, 1.0);
    })";
}

const char* OcclusionRenderer::compose_fragment_src() {
    return R"(#version 460 core
    in vec2 v_uv;
    uniform sampler2D u_depthTex;
    uniform float u_near;
    uniform float u_far;
    uniform float u_handDepth;
    uniform float u_strength;
    out vec4 fragColor;
    void main() {
        float d = texture(u_depthTex, v_uv).r;
        // If the hand is closer than the object, discard the object pixel
        if (d < u_handDepth && d > 0.0) {
            discard;  // Hand occludes the object
        }
        fragColor = vec4(0);  // Pass through
    })";
}

const char* OcclusionRenderer::chroma_vertex_src() { return compose_vertex_src(); }

const char* OcclusionRenderer::chroma_fragment_src() {
    return R"(#version 460 core
    in vec2 v_uv;
    uniform sampler2D u_bgTex;
    uniform vec3 u_chromaColor;
    uniform float u_threshold;
    out vec4 fragColor;
    void main() {
        vec4 color = texture(u_bgTex, v_uv);
        float diff = length(color.rgb - u_chromaColor);
        if (diff < u_threshold) {
            discard;  // Chroma key: make transparent
        }
        fragColor = color;
    })";
}

} // namespace hlp
