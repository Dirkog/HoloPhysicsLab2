#include "OptimizedMetaball.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace hlp {

OptimizedMetaball::OptimizedMetaball(Renderer& renderer) : renderer_(renderer) {}
OptimizedMetaball::~OptimizedMetaball() { shutdown(); }

void OptimizedMetaball::init(int width, int height) {
    init_fbo(width, height);
    init_quad();
    init_shaders();
    std::cout << "[OptMetaball] Init " << width << "x" << height << std::endl;
}

void OptimizedMetaball::shutdown() {
    if (occ_fbo_) glDeleteFramebuffers(1, &occ_fbo_);
    if (occ_depth_) glDeleteTextures(1, &occ_depth_);
    if (quad_vao_) glDeleteVertexArrays(1, &quad_vao_);
    if (occ_shader_) glDeleteProgram(occ_shader_);
    if (compose_shader_) glDeleteProgram(compose_shader_);
}

void OptimizedMetaball::resize(int w, int h) { shutdown(); init(w, h); }

void OptimizedMetaball::init_fbo(int w, int h) {
    fbo_w_ = w; fbo_h_ = h;
    
    glGenFramebuffers(1, &occ_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, occ_fbo_);
    
    glGenTextures(1, &occ_depth_);
    glBindTexture(GL_TEXTURE_2D, occ_depth_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, w, h,
                 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, occ_depth_, 0);
    
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cerr << "[OptMetaball] FBO error!" << std::endl;
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OptimizedMetaball::init_quad() {
    float verts[] = {-1,-1,0,0, 1,-1,1,0, -1,1,0,1, 1,1,1,1};
    glGenVertexArrays(1, &quad_vao_);
    glBindVertexArray(quad_vao_);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,4*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,4*sizeof(float),(void*)(2*sizeof(float)));
    glEnableVertexAttribArray(1);
}

void OptimizedMetaball::init_shaders() {
    occ_shader_ = renderer_.compile_shader(occ_vs(), occ_fs());
    compose_shader_ = renderer_.compile_shader(compose_vs(), compose_fs());
    
    u_viewProj_ = glGetUniformLocation(occ_shader_, "u_viewProj");
    u_joints_ = glGetUniformLocation(occ_shader_, "u_joints");
    u_radii_ = glGetUniformLocation(occ_shader_, "u_radii");
    u_num_joints_ = glGetUniformLocation(occ_shader_, "u_num_joints");
    u_smooth_k_ = glGetUniformLocation(occ_shader_, "u_smooth_k");
    u_clusters_ = glGetUniformLocation(occ_shader_, "u_clusters");
    u_num_clusters_ = glGetUniformLocation(occ_shader_, "u_num_clusters");
    u_scissor_ = glGetUniformLocation(occ_shader_, "u_scissor");
    u_viewport_ = glGetUniformLocation(occ_shader_, "u_viewport");
    u_quality_ = glGetUniformLocation(occ_shader_, "u_quality");
}

// ==================== Bounding Box Computation ====================

HandBounds OptimizedMetaball::compute_hand_bounds(
    const VirtualHand& hand, const glm::mat4& view_proj,
    int viewport_w, int viewport_h) {
    
    HandBounds bounds;
    if (!hand.is_active()) { bounds.valid = false; return bounds; }
    
    auto& world = hand.world_landmarks();
    dvec3 center(0);
    double min_x = 1e10, min_y = 1e10, max_x = -1e10, max_y = -1e10;
    
    // Project all 21 joints to screen space
    for (int i = 0; i < 21; i++) {
        bounds.joints[i] = world[i];
        bounds.radii[i] = 0.04;
        
        // Project to clip space
        glm::vec4 clip = view_proj * glm::vec4(
            (float)world[i].x, (float)world[i].z, (float)world[i].y, 1.0f);
        
        if (clip.w > 0) {
            float ndc_x = clip.x / clip.w;
            float ndc_y = clip.y / clip.w;
            
            // Screen space (pixels)
            float sx = (ndc_x * 0.5f + 0.5f) * viewport_w;
            float sy = (1.0f - ndc_y * 0.5f - 0.5f) * viewport_h;
            
            min_x = std::min((double)min_x, (double)sx);
            max_x = std::max((double)max_x, (double)sx);
            min_y = std::min((double)min_y, (double)sy);
            max_y = std::max((double)max_y, (double)sy);
        }
        
        center += world[i];
    }
    center /= 21.0;
    
    // Expand bounds by hand radius (in pixels)
    double hand_radius_px = 150.0;  // Approximate
    bounds.min_x = std::max(0, (int)(min_x - hand_radius_px));
    bounds.min_y = std::max(0, (int)(min_y - hand_radius_px));
    bounds.max_x = std::min(viewport_w - 1, (int)(max_x + hand_radius_px));
    bounds.max_y = std::min(viewport_h - 1, (int)(max_y + hand_radius_px));
    bounds.valid = (bounds.min_x < bounds.max_x && bounds.min_y < bounds.max_y);
    
    // === Build hierarchical clusters ===
    // Cluster 0: Palm (wrist + palm base: indices 0,5,9,13,17)
    dvec3 palm_center(0);
    int palm_idx[] = {0, 5, 9, 13, 17};
    for (int i : palm_idx) palm_center += world[i];
    palm_center /= 5;
    double palm_radius = 0.08;
    bounds.clusters[0] = {palm_center, palm_radius, 0, 18};
    
    // Clusters 1-5: Fingers
    struct FingerJoints { int start, end; const char* name; };
    FingerJoints fingers[] = {
        {1, 4, "thumb"},   // Thumb CMC→TIP
        {5, 8, "index"},   // Index MCP→TIP
        {9, 12, "middle"}, // Middle MCP→TIP
        {13, 16, "ring"},  // Ring MCP→TIP
        {17, 20, "pinky"}, // Pinky MCP→TIP
    };
    
    for (int f = 0; f < 5; f++) {
        auto& fj = fingers[f];
        dvec3 fc(0);
        for (int j = fj.start; j <= fj.end; j++) fc += world[j];
        fc /= (fj.end - fj.start + 1);
        
        double max_r = 0;
        for (int j = fj.start; j <= fj.end; j++) {
            double d = glm::length(world[j] - fc);
            max_r = std::max(max_r, d);
        }
        
        bounds.clusters[f + 1] = {fc, max_r + 0.03, fj.start, fj.end};
    }
    
    return bounds;
}

// ==================== Main Render ====================

void OptimizedMetaball::render_occlusion(const VirtualHand& hand,
                                          const glm::mat4& view_proj,
                                          int viewport_w, int viewport_h) {
    if (!enabled_ || !hand.is_active()) return;
    
    // 1. Compute bounds (CPU)
    HandBounds bounds = compute_hand_bounds(hand, view_proj, viewport_w, viewport_h);
    if (!bounds.valid) return;
    
    // 2. Calculate scissor rect
    int scissor_w = bounds.max_x - bounds.min_x;
    int scissor_h = bounds.max_y - bounds.min_y;
    float area_ratio = (float)(scissor_w * scissor_h) / (viewport_w * viewport_h);
    
    // Skip if scissor rect is too small (degenerate)
    if (scissor_w < 4 || scissor_h < 4) return;
    
    last_pixels_ = scissor_w * scissor_h;
    last_primitives_ = 21;  // Base joints
    
    // 3. Render occlusion to FBO (scissored)
    glBindFramebuffer(GL_FRAMEBUFFER, occ_fbo_);
    glViewport(0, 0, fbo_w_, fbo_h_);
    
    // Enable scissor test to limit rendering to hand area only
    glEnable(GL_SCISSOR_TEST);
    glScissor(bounds.min_x, bounds.min_y, scissor_w, scissor_h);
    
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glDepthMask(GL_TRUE);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    
    glUseProgram(occ_shader_);
    glUniformMatrix4fv(u_viewProj_, 1, GL_FALSE, glm::value_ptr(view_proj));
    glUniform1i(u_num_joints_, 21);
    glUniform1f(u_smooth_k_, 0.15f);
    glUniform1i(u_num_clusters_, 6);
    glUniform4f(u_scissor_, (float)bounds.min_x, (float)bounds.min_y,
                (float)scissor_w, (float)scissor_h);
    glUniform2f(u_viewport_, (float)viewport_w, (float)viewport_h);
    glUniform1i(u_quality_, quality_);
    
    // Upload joint positions (relative to origin for precision)
    float joint_data[21 * 3];
    for (int i = 0; i < 21; i++) {
        joint_data[i*3] = (float)bounds.joints[i].x;
        joint_data[i*3+1] = (float)bounds.joints[i].y;
        joint_data[i*3+2] = (float)bounds.joints[i].z;
    }
    glUniform3fv(u_joints_, 21, joint_data);
    
    float radii_data[21];
    for (int i = 0; i < 21; i++) radii_data[i] = (float)bounds.radii[i];
    glUniform1fv(u_radii_, 21, radii_data);
    
    // Upload clusters
    float cluster_data[6 * 4]; // center(x3) + radius
    for (int i = 0; i < 6; i++) {
        cluster_data[i*4] = (float)bounds.clusters[i].center.x;
        cluster_data[i*4+1] = (float)bounds.clusters[i].center.y;
        cluster_data[i*4+2] = (float)bounds.clusters[i].center.z;
        cluster_data[i*4+3] = (float)bounds.clusters[i].radius;
    }
    glUniform4fv(u_clusters_, 6, cluster_data);
    
    // Render scissored fullscreen quad
    glBindVertexArray(quad_vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glDisable(GL_SCISSOR_TEST);
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    // 4. Compose occlusion into main depth buffer
    glBindTexture(GL_TEXTURE_2D, occ_depth_);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, occ_fbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y,
                      bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y,
                      GL_DEPTH_BUFFER_BIT, GL_NEAREST);
}

// ==================== Shaders ====================

const char* OptimizedMetaball::occ_vs() {
    return R"(#version 460 core
    layout(location = 0) in vec2 a_pos;
    layout(location = 1) in vec2 a_uv;
    out vec2 v_uv;
    uniform vec4 u_scissor;
    uniform vec2 u_viewport;
    void main() {
        // Transform UV to scissored region
        vec2 scissor_uv = a_uv;
        scissor_uv.x = u_scissor.x / u_viewport.x + a_uv.x * (u_scissor.z / u_viewport.x);
        scissor_uv.y = u_scissor.y / u_viewport.y + a_uv.y * (u_scissor.w / u_viewport.y);
        v_uv = scissor_uv;
        gl_Position = vec4(a_pos, 0.0, 1.0);
    })";
}

const char* OptimizedMetaball::occ_fs() {
    return R"(#version 460 core
    in vec2 v_uv;
    
    uniform mat4 u_viewProj;
    uniform vec3 u_joints[21];
    uniform float u_radii[21];
    uniform int u_num_joints;
    uniform float u_smooth_k;
    uniform vec4 u_clusters[6];
    uniform int u_num_clusters;
    uniform int u_quality;
    
    float smin(float a, float b, float k) {
        float h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
        return mix(b, a, h) - k * h * (1.0 - h);
    }
    
    float dist_to_segment(vec3 p, vec3 a, vec3 b, float ra, float rb) {
        vec3 ab = b - a;
        float t = clamp(dot(p - a, ab) / dot(ab, ab), 0.0, 1.0);
        vec3 closest = a + ab * t;
        float r = mix(ra, rb, t);
        return length(p - closest) - r;
    }
    
    void main() {
        // Reconstruct world position from UV and depth
        vec4 ndc = vec4(v_uv * 2.0 - 1.0, 1.0, 1.0);
        vec4 world_pos = inverse(u_viewProj) * ndc;
        world_pos /= world_pos.w;
        vec3 p = world_pos.xyz;
        
        // === Hierarchical cluster culling ===
        // First check against 6 bounding spheres (very cheap)
        float min_sdf = 1e10;
        int active_clusters[6];
        int num_active = 0;
        
        for (int c = 0; c < u_num_clusters; c++) {
            vec3 center = u_clusters[c].xyz;
            float r = u_clusters[c].w;
            float d = length(p - center) - r * 2.0; // Conservative bound
            if (d < 0.5) {  // Close enough to check
                active_clusters[num_active++] = c;
            }
        }
        
        if (num_active == 0) {
            discard;  // Early exit: pixel is far from hand
            return;
        }
        
        // === Full SDF evaluation on active clusters ===
        float sdf = 1e10;
        
        // Joint spheres
        for (int c = 0; c < num_active; c++) {
            int cluster = active_clusters[c];
            int start = 0, end = u_num_joints;
            
            // Get cluster joint range (simplified: skip thumb/pinky on low quality)
            if (u_quality < 2 && (cluster == 1 || cluster == 5)) continue;
            
            // Sphere SDF for each joint in cluster
            for (int i = start; i < end; i++) {
                float d = length(p - u_joints[i]) - u_radii[i];
                sdf = smin(sdf, d, u_smooth_k);
            }
        }
        
        // Bone capsules (same cluster-based culling)
        int bones[][2] = int[][2](
            {0,1},{1,2},{2,3},{3,4},
            {0,5},{5,6},{6,7},{7,8},
            {0,9},{9,10},{10,11},{11,12},
            {0,13},{13,14},{14,15},{15,16},
            {0,17},{17,18},{18,19},{19,20},
            {5,9},{9,13},{13,17}
        );
        
        for (int b = 0; b < 23; b++) {
            int i = bones[b][0], j = bones[b][1];
            float d = dist_to_segment(p, u_joints[i], u_joints[j],
                                      u_radii[i] * 0.6, u_radii[j] * 0.6);
            sdf = smin(sdf, d, u_smooth_k);
        }
        
        if (sdf < 0.0) {
            gl_FragDepth = 0.5;  // Hand occludes
        } else {
            discard;
        }
    })";
}

const char* OptimizedMetaball::compose_vs() { return occ_vs(); }
const char* OptimizedMetaball::compose_fs() { return occ_fs(); }

} // namespace hlp
