#include "ShadowCatcher.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <algorithm>

namespace hlp {

ShadowCatcher::ShadowCatcher(Renderer& renderer) : renderer_(renderer) {}
ShadowCatcher::~ShadowCatcher() { shutdown(); }

void ShadowCatcher::init(int shadow_map_size) {
    shadow_size_ = shadow_map_size;
    init_fbo();
    init_shader();
    init_compose_quad();
    std::cout << "[ShadowCatcher] Init " << shadow_map_size << "×" << shadow_map_size << std::endl;
}

void ShadowCatcher::shutdown() {
    if (shadow_fbo_) glDeleteFramebuffers(1, &shadow_fbo_);
    if (shadow_depth_tex_) glDeleteTextures(1, &shadow_depth_tex_);
    if (shadow_shader_) glDeleteProgram(shadow_shader_);
    if (compose_vao_) glDeleteVertexArrays(1, &compose_vao_);
}

void ShadowCatcher::init_fbo() {
    glGenFramebuffers(1, &shadow_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, shadow_fbo_);
    
    glGenTextures(1, &shadow_depth_tex_);
    glBindTexture(GL_TEXTURE_2D, shadow_depth_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, 
                 shadow_size_, shadow_size_, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, shadow_depth_tex_, 0);
    
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cerr << "[ShadowCatcher] FBO error!" << std::endl;
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ShadowCatcher::init_shader() {
    shadow_shader_ = renderer_.compile_shader(shadow_vs(), shadow_fs());
}

void ShadowCatcher::init_compose_quad() {
    float verts[] = {-1,-1,0,0, 1,-1,1,0, -1,1,0,1, 1,1,1,1};
    glGenVertexArrays(1, &compose_vao_);
    glBindVertexArray(compose_vao_);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,4*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,4*sizeof(float),(void*)(2*sizeof(float)));
    glEnableVertexAttribArray(1);
}

glm::mat4 ShadowCatcher::light_view_matrix() const {
    return glm::lookAt(
        light_dir_ * 30.0f,         // Light position (far away)
        light_target_,               // Look at scene center
        glm::vec3(0.0f, 1.0f, 0.0f) // Up vector
    );
}

glm::mat4 ShadowCatcher::light_proj_matrix() const {
    // Orthographic projection for directional light
    float size = 20.0f;
    return glm::ortho(-size, size, -size, size, 1.0f, 50.0f);
}

void ShadowCatcher::begin_shadow_pass() {
    glBindFramebuffer(GL_FRAMEBUFFER, shadow_fbo_);
    glViewport(0, 0, shadow_size_, shadow_size_);
    glClear(GL_DEPTH_BUFFER_BIT);
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    
    // Update light VP
    light_vp_ = light_proj_matrix() * light_view_matrix();
}

void ShadowCatcher::end_shadow_pass() {
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ShadowCatcher::render_shadow(const std::vector<Body>& bodies,
                                   const glm::mat4& view_proj,
                                   const glm::mat4& light_vp) {
    (void)view_proj;
    if (!config_.enabled) return;
    
    // Render bodies into shadow map
    glUseProgram(shadow_shader_);
    glUniformMatrix4fv(glGetUniformLocation(shadow_shader_, "u_lightVP"),
                       1, GL_FALSE, glm::value_ptr(light_vp));
    
    for (const auto& body : bodies) {
        if (body.is_static()) continue;
        
        glm::mat4 model = glm::translate(glm::mat4(1.0f),
            glm::vec3((float)body.state.position.x, (float)body.state.position.z,
                      (float)body.state.position.y));
        model = glm::scale(model, glm::vec3((float)body.props.radius));
        
        glUniformMatrix4fv(glGetUniformLocation(shadow_shader_, "u_model"),
                           1, GL_FALSE, glm::value_ptr(model));
        
        // Use sphere VAO (simplified: all shapes cast sphere shadows)
        GLuint vao = 0; // Need to get from Renderer
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 36); // Use appropriate count
    }
}

void ShadowCatcher::render_shadows(const std::vector<Body>& bodies,
                                    const glm::mat4& view_proj) {
    if (!config_.enabled) return;
    
    begin_shadow_pass();
    render_shadow(bodies, view_proj, light_vp_);
    end_shadow_pass();
    
    // Compose shadow onto background
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Use the shadow compose shader
    GLuint compose_shader = 0; // Need compose shader
    glUseProgram(compose_shader);
    
    glUniformMatrix4fv(glGetUniformLocation(compose_shader, "u_viewProj"),
                       1, GL_FALSE, glm::value_ptr(view_proj));
    glUniformMatrix4fv(glGetUniformLocation(compose_shader, "u_lightVP"),
                       1, GL_FALSE, glm::value_ptr(light_vp_));
    glUniform1i(glGetUniformLocation(compose_shader, "u_shadowMap"), 0);
    glUniform1f(glGetUniformLocation(compose_shader, "u_intensity"), 
                config_.shadow_intensity);
    glUniform3fv(glGetUniformLocation(compose_shader, "u_groundNormal"),
                 1, &config_.ground_normal[0]);
    glUniform1f(glGetUniformLocation(compose_shader, "u_groundOffset"),
                config_.ground_offset);
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadow_depth_tex_);
    
    glBindVertexArray(compose_vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

void ShadowCatcher::calibrate_ground_plane(const dvec3& point_on_ground,
                                             const dvec3& normal) {
    config_.ground_normal = glm::vec3((float)normal.x, (float)normal.z, (float)normal.y);
    config_.ground_offset = glm::dot(
        glm::vec3((float)point_on_ground.x, (float)point_on_ground.z, (float)point_on_ground.y),
        config_.ground_normal);
    std::cout << "[ShadowCatcher] Ground plane calibrated" << std::endl;
}

void ShadowCatcher::estimate_light_direction(const cv::Mat& camera_frame,
                                              const std::vector<dvec3>& hand_landmarks_2d) {
    if (!config_.auto_light_direction || camera_frame.empty()) return;
    
    glm::vec3 gradient = estimate_from_gradient(camera_frame);
    if (glm::length(gradient) > 0.1f) {
        light_dir_ = glm::normalize(gradient);
        config_.light_direction = light_dir_;
    }
}

glm::vec3 ShadowCatcher::estimate_from_gradient(const cv::Mat& frame) {
    // Simplified: use average gradient direction
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
    
    cv::Mat grad_x, grad_y;
    cv::Scharr(gray, grad_x, CV_32F, 1, 0);
    cv::Scharr(gray, grad_y, CV_32F, 0, 1);
    
    float mean_dx = cv::mean(grad_x)[0];
    float mean_dy = cv::mean(grad_y)[0];
    
    // Light comes from the direction of the brightest region
    return glm::vec3(-mean_dx, -mean_dy, 1.0f);
}

// ==================== Shaders ====================

const char* ShadowCatcher::shadow_vs() {
    return R"(#version 460 core
    layout(location = 0) in vec3 a_pos;
    uniform mat4 u_model;
    uniform mat4 u_lightVP;
    void main() {
        gl_Position = u_lightVP * u_model * vec4(a_pos, 1.0);
    })";
}

const char* ShadowCatcher::shadow_fs() {
    return R"(#version 460 core
    void main() {
        // Depth is written automatically
    })";
}

const char* ShadowCatcher::compose_vs() {
    return R"(#version 460 core
    layout(location = 0) in vec2 a_pos;
    layout(location = 1) in vec2 a_uv;
    out vec2 v_uv;
    void main() {
        v_uv = a_uv;
        gl_Position = vec4(a_pos, 0.0, 1.0);
    })";
}

const char* ShadowCatcher::compose_fs() {
    return R"(#version 460 core
    in vec2 v_uv;
    uniform mat4 u_viewProj;
    uniform mat4 u_lightVP;
    uniform sampler2D u_shadowMap;
    uniform float u_intensity;
    uniform vec3 u_groundNormal;
    uniform float u_groundOffset;
    out vec4 fragColor;
    
    float shadow_bias = 0.001;
    
    float sample_shadow(vec3 world_pos) {
        vec4 light_pos = u_lightVP * vec4(world_pos, 1.0);
        vec3 proj = light_pos.xyz / light_pos.w;
        proj = proj * 0.5 + 0.5;
        
        if (proj.x < 0 || proj.x > 1 || proj.y < 0 || proj.y > 1 || proj.z > 1)
            return 1.0;
        
        float closest = texture(u_shadowMap, proj.xy).r;
        float current = proj.z;
        
        // PCF comparison
        float shadow = 0.0;
        vec2 texel = 1.0 / textureSize(u_shadowMap, 0);
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                float pcf = texture(u_shadowMap, proj.xy + vec2(x, y) * texel).r;
                shadow += (current - shadow_bias) > pcf ? u_intensity : 0.0;
            }
        }
        return 1.0 - shadow / 9.0;
    }
    
    void main() {
        // Reconstruct world position from depth
        vec4 ndc = vec4(v_uv * 2.0 - 1.0, 0.5, 1.0);
        vec4 world = inverse(u_viewProj) * ndc;
        world /= world.w;
        
        // Project onto ground plane
        float dist = dot(world.xyz, u_groundNormal) - u_groundOffset;
        vec3 ground_pos = world.xyz - u_groundNormal * dist;
        
        // Sample shadow at ground position
        float light = sample_shadow(ground_pos);
        
        // Output: darken the background where shadow falls
        fragColor = vec4(0.0, 0.0, 0.0, (1.0 - light) * u_intensity);
    })";
}

} // namespace hlp
