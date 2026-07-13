#include "Renderer.h"
#include <iostream>
#include <cstring>
#include <cmath>

// Minimal OpenGL headers are included via glad.h in main.cpp
// For standalone compilation we declare the functions we use
extern "C" {
    extern void glGenVertexArrays(int, unsigned int*);
    extern void glBindVertexArray(unsigned int);
    extern void glGenBuffers(int, unsigned int*);
    extern void glBindBuffer(int, unsigned int);
    extern void glBufferData(int, long, const void*, int);
    extern void glEnableVertexAttribArray(unsigned int);
    extern void glVertexAttribPointer(unsigned int, int, int, int, int, const void*);
    extern void glDrawArrays(int, int, int);
    extern void glUseProgram(unsigned int);
    extern void glUniformMatrix4fv(int, int, int, const float*);
    extern void glUniform3fv(int, int, const float*);
    extern void glUniform1f(int, float);
    extern void glUniform1i(int, int);
    extern void glClear(int);
    extern void glClearColor(float, float, float, float);
    extern void glViewport(int, int, int, int);
    extern void glEnable(int);
    extern void glDisable(int);
    extern void glDepthMask(int);
    extern void glLineWidth(float);
    extern void glPointSize(float);
    extern void glBlendFunc(int, int);
}

namespace hlp {

Renderer::Renderer() = default;
Renderer::~Renderer() { shutdown(); }

bool Renderer::init(int width, int height) {
    width_ = width;
    height_ = height;
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    
    // Generate all mesh geometry
    generate_sphere(vao_sphere_, sphere_verts_);
    generate_box(vao_box_, box_verts_);
    generate_cylinder(vao_cylinder_, cylinder_verts_);
    generate_torus(vao_torus_, torus_verts_);
    generate_grid(vao_grid_, grid_verts_);
    
    // Compile shaders
    shader_body_ = compile_shader(body_vertex_src(), body_fragment_src());
    shader_trail_ = compile_shader(trail_vertex_src(), trail_fragment_src());
    shader_grid_ = compile_shader(grid_vertex_src(), grid_fragment_src());
    shader_arrow_ = compile_shader(arrow_vertex_src(), arrow_fragment_src());
    shader_ring_ = compile_shader(grid_vertex_src(), grid_fragment_src());
    
    return true;
}

void Renderer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    glViewport(0, 0, width_, height_);
}

void Renderer::shutdown() {
    if (vao_sphere_) { glDeleteVertexArrays(1, &vao_sphere_); vao_sphere_ = 0; }
    if (vao_box_) { glDeleteVertexArrays(1, &vao_box_); vao_box_ = 0; }
    if (vao_cylinder_) { glDeleteVertexArrays(1, &vao_cylinder_); vao_cylinder_ = 0; }
    if (vao_torus_) { glDeleteVertexArrays(1, &vao_torus_); vao_torus_ = 0; }
    if (vao_grid_) { glDeleteVertexArrays(1, &vao_grid_); vao_grid_ = 0; }
    
    if (shader_body_) glDeleteProgram(shader_body_);
    if (shader_trail_) glDeleteProgram(shader_trail_);
    if (shader_grid_) glDeleteProgram(shader_grid_);
    if (shader_arrow_) glDeleteProgram(shader_arrow_);
}

void Renderer::begin_frame() {
    glClearColor(clear_color_.r, clear_color_.g, clear_color_.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::end_frame() {
    // Swap buffers handled by main
}

void Renderer::set_camera(const glm::vec3& eye, const glm::vec3& target) {
    camera_eye_ = eye;
    camera_target_ = target;
}

void Renderer::orbit_camera(float dx, float dy) {
    camera_theta_ += dx;
    camera_phi_ = glm::clamp(camera_phi_ + dy, 0.1f, glm::half_pi<float>() - 0.01f);
    
    camera_eye_.x = camera_target_.x + camera_distance_ * std::sin(camera_phi_) * std::cos(camera_theta_);
    camera_eye_.y = camera_target_.y + camera_distance_ * std::cos(camera_phi_);
    camera_eye_.z = camera_target_.z + camera_distance_ * std::sin(camera_phi_) * std::sin(camera_theta_);
}

void Renderer::zoom_camera(float delta) {
    camera_distance_ = glm::clamp(camera_distance_ * (1.0f - delta), 1.0f, 100.0f);
    orbit_camera(0, 0); // Recalculate position
}

void Renderer::pan_camera(float dx, float dy) {
    // Simple pan by moving target
    glm::vec3 forward = glm::normalize(camera_target_ - camera_eye_);
    glm::vec3 right = glm::normalize(glm::cross(forward, camera_up_));
    glm::vec3 up = glm::normalize(glm::cross(right, forward));
    
    camera_target_ += right * dx * camera_distance_ * 0.01f;
    camera_target_ += up * dy * camera_distance_ * 0.01f;
    orbit_camera(0, 0);
}

glm::mat4 Renderer::view_matrix() const {
    return glm::lookAt(camera_eye_, camera_target_, camera_up_);
}

glm::mat4 Renderer::proj_matrix() const {
    return glm::perspective(glm::radians(fov_), (float)width_ / (float)height_, near_plane_, far_plane_);
}

void Renderer::render_scene(const std::vector<Body>& bodies, const SimulationParams& params,
                             const std::vector<CollisionEvent>& events) {
    auto view = view_matrix();
    auto proj = proj_matrix();
    auto vp = proj * view;
    
    // Render grid
    if (params.show_grid) {
        render_grid(params);
    }
    
    // Render trails
    if (params.show_trails) {
        render_trails(bodies);
    }
    
    // Render bodies
    glUseProgram(shader_body_);
    glUniformMatrix4fv(glGetUniformLocation(shader_body_, "u_viewProj"), 1, GL_FALSE, &vp[0][0]);
    glUniform3fv(glGetUniformLocation(shader_body_, "u_cameraPos"), 1, &camera_eye_[0]);
    
    // Ambient + directional light
    glUniform3f(glGetUniformLocation(shader_body_, "u_lightDir"), 0.3f, 0.8f, 0.4f);
    glUniform3f(glGetUniformLocation(shader_body_, "u_lightColor"), 0.9f, 0.9f, 1.0f);
    glUniform3f(glGetUniformLocation(shader_body_, "u_ambient"), 0.15f, 0.15f, 0.18f);
    
    for (const auto& body : bodies) {
        if (!body.is_visible()) continue;
        
        // Model matrix
        glm::mat4 model = glm::translate(glm::mat4(1.0f), 
            glm::vec3(body.state.position.x, body.state.position.z, body.state.position.y));
        model = glm::scale(model, glm::vec3((float)body.props.radius));
        
        glUniformMatrix4fv(glGetUniformLocation(shader_body_, "u_model"), 1, GL_FALSE, &model[0][0]);
        glUniform3fv(glGetUniformLocation(shader_body_, "u_color"), 1, &body.visual.color[0]);
        glUniform3fv(glGetUniformLocation(shader_body_, "u_glowColor"), 1, &body.visual.glow_color[0]);
        glUniform1f(glGetUniformLocation(shader_body_, "u_emissive"), body.visual.emissive_intensity);
        glUniform1f(glGetUniformLocation(shader_body_, "u_opacity"), body.visual.opacity);
        glUniform1i(glGetUniformLocation(shader_body_, "u_selected"), body.id == selected_body_id_ ? 1 : 0);
        
        // Select mesh based on shape
        GLuint vao = vao_sphere_;
        int count = sphere_verts_;
        switch (body.visual.shape) {
            case ShapeType::Box:       vao = vao_box_; count = box_verts_; break;
            case ShapeType::Cylinder:  vao = vao_cylinder_; count = cylinder_verts_; break;
            case ShapeType::Torus:     vao = vao_torus_; count = torus_verts_; break;
            default: break; // sphere
        }
        
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, count);
    }
    
    // Render collision events
    render_collision_events(events, 0);
}

void Renderer::render_grid(const SimulationParams& params) {
    auto vp = proj_matrix() * view_matrix();
    
    glUseProgram(shader_grid_);
    glUniformMatrix4fv(glGetUniformLocation(shader_grid_, "u_viewProj"), 1, GL_FALSE, &vp[0][0]);
    
    glBindVertexArray(vao_grid_);
    glDrawArrays(GL_LINES, 0, grid_verts_);
}

void Renderer::render_trails(const std::vector<Body>& bodies) {
    // Simplified trail rendering using line segments
    // In a full implementation this would use a dedicated trail system
    glUseProgram(shader_trail_);
    auto vp = proj_matrix() * view_matrix();
    glUniformMatrix4fv(glGetUniformLocation(shader_trail_, "u_viewProj"), 1, GL_FALSE, &vp[0][0]);
    
    for (const auto& body : bodies) {
        if (body.trail.size() < 2) continue;
        
        // For each body, we'd draw a line strip
        // Simplified: skip for now, full impl would batch trails
    }
}

void Renderer::render_forces(const std::vector<Body>& bodies) {
    // TODO: implement force arrows
}

void Renderer::render_collision_events(const std::vector<CollisionEvent>& events, double time) {
    // Render small expanding rings at collision points
    if (events.empty()) return;
    
    auto vp = proj_matrix() * view_matrix();
    
    glUseProgram(shader_ring_);
    glUniformMatrix4fv(glGetUniformLocation(shader_ring_, "u_viewProj"), 1, GL_FALSE, &vp[0][0]);
    glDisable(GL_DEPTH_TEST);
    
    for (const auto& e : events) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), 
            glm::vec3((float)e.point.x, (float)e.point.z, (float)e.point.y));
        model = glm::scale(model, glm::vec3(0.3f));
        glUniformMatrix4fv(glGetUniformLocation(shader_ring_, "u_model"), 1, GL_FALSE, &model[0][0]);
        glUniform3f(glGetUniformLocation(shader_ring_, "u_color"), 1.0f, 0.2f, 0.2f);
        
        // Draw a small sphere as collision marker using the sphere VAO
        glBindVertexArray(vao_sphere_);
        glDrawArrays(GL_TRIANGLES, 0, sphere_verts_);
    }
    
    glEnable(GL_DEPTH_TEST);
}

void Renderer::render_selection_highlight(const Body* selected) {
    // Could add a glow ring around selected body
}

int Renderer::pick_body(int screen_x, int screen_y, int viewport_w, int viewport_h,
                         const std::vector<Body>& bodies) {
    // Simplified picking: cast a ray and check sphere intersections
    // Convert screen coords to NDC
    float ndc_x = (2.0f * screen_x / viewport_w) - 1.0f;
    float ndc_y = 1.0f - (2.0f * screen_y / viewport_h);
    
    glm::mat4 inv_vp = glm::inverse(proj_matrix() * view_matrix());
    
    // Ray in world space
    glm::vec4 near_pt = inv_vp * glm::vec4(ndc_x, ndc_y, -1.0f, 1.0f);
    glm::vec4 far_pt = inv_vp * glm::vec4(ndc_x, ndc_y, 1.0f, 1.0f);
    
    near_pt /= near_pt.w;
    far_pt /= far_pt.w;
    
    glm::vec3 ray_origin = glm::vec3(near_pt);
    glm::vec3 ray_dir = glm::normalize(glm::vec3(far_pt) - ray_origin);
    
    // Check sphere intersections
    int closest_id = -1;
    float closest_t = 1e10f;
    
    for (const auto& body : bodies) {
        glm::vec3 body_pos((float)body.state.position.x, (float)body.state.position.z, (float)body.state.position.y);
        float radius = (float)body.props.radius;
        
        glm::vec3 oc = ray_origin - body_pos;
        float a = glm::dot(ray_dir, ray_dir);
        float b = 2.0f * glm::dot(oc, ray_dir);
        float c = glm::dot(oc, oc) - radius * radius;
        float disc = b * b - 4.0f * a * c;
        
        if (disc > 0) {
            float t = (-b - std::sqrt(disc)) / (2.0f * a);
            if (t > 0 && t < closest_t) {
                closest_t = t;
                closest_id = (int)body.id;
            }
        }
    }
    
    return closest_id;
}

// ==================== Geometry Generation ====================
void Renderer::generate_sphere(GLuint& vao, int& vert_count, float radius, int slices, int stacks) {
    std::vector<float> vertices;
    
    for (int i = 0; i < stacks; ++i) {
        float phi1 = (float)i / stacks * (float)M_PI;
        float phi2 = (float)(i + 1) / stacks * (float)M_PI;
        
        for (int j = 0; j < slices; ++j) {
            float theta1 = (float)j / slices * 2.0f * (float)M_PI;
            float theta2 = (float)(j + 1) / slices * 2.0f * (float)M_PI;
            
            auto add_vert = [&](float phi, float theta) {
                float x = radius * std::sin(phi) * std::cos(theta);
                float y = radius * std::cos(phi);
                float z = radius * std::sin(phi) * std::sin(theta);
                vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                // Normal
                float len = std::sqrt(x*x + y*y + z*z);
                vertices.push_back(x/len); vertices.push_back(y/len); vertices.push_back(z/len);
            };
            
            // Triangle 1
            add_vert(phi1, theta1);
            add_vert(phi1, theta2);
            add_vert(phi2, theta1);
            
            // Triangle 2
            add_vert(phi1, theta2);
            add_vert(phi2, theta2);
            add_vert(phi2, theta1);
        }
    }
    
    vert_count = (int)vertices.size() / 6;
    
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
}

void Renderer::generate_box(GLuint& vao, int& vert_count, float size) {
    float h = size * 0.5f;
    // 12 triangles = 36 vertices (6 faces * 2 triangles * 3 verts)
    float verts[] = {
        -h,-h,-h, 0,0,-1,  h,-h,-h, 0,0,-1,  h, h,-h, 0,0,-1,
        -h,-h,-h, 0,0,-1,  h, h,-h, 0,0,-1, -h, h,-h, 0,0,-1,
        -h,-h, h, 0,0,1,  h, h, h, 0,0,1,  h,-h, h, 0,0,1,
        -h,-h, h, 0,0,1, -h, h, h, 0,0,1,  h, h, h, 0,0,1,
        -h,-h,-h, -1,0,0, -h, h,-h, -1,0,0, -h, h, h, -1,0,0,
        -h,-h,-h, -1,0,0, -h, h, h, -1,0,0, -h,-h, h, -1,0,0,
        h,-h,-h, 1,0,0, h, h, h, 1,0,0, h, h,-h, 1,0,0,
        h,-h,-h, 1,0,0, h,-h, h, 1,0,0, h, h, h, 1,0,0,
        -h,-h,-h, 0,-1,0, h,-h, h, 0,-1,0, h,-h,-h, 0,-1,0,
        -h,-h,-h, 0,-1,0, -h,-h, h, 0,-1,0, h,-h, h, 0,-1,0,
        -h, h,-h, 0,1,0, h, h,-h, 0,1,0, h, h, h, 0,1,0,
        -h, h,-h, 0,1,0, h, h, h, 0,1,0, -h, h, h, 0,1,0,
    };
    
    vert_count = 36;
    
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
}

void Renderer::generate_cylinder(GLuint& vao, int& vert_count, float radius, float height, int slices) {
    std::vector<float> verts;
    float hh = height * 0.5f;
    
    for (int i = 0; i < slices; ++i) {
        float a1 = (float)i / slices * 2.0f * (float)M_PI;
        float a2 = (float)(i+1) / slices * 2.0f * (float)M_PI;
        float x1 = radius * std::cos(a1), z1 = radius * std::sin(a1);
        float x2 = radius * std::cos(a2), z2 = radius * std::sin(a2);
        
        // Side triangles
        verts.insert(verts.end(), { x1,-hh,z1, x1/std::sqrt(x1*x1+z1*z1),0,z1/std::sqrt(x1*x1+z1*z1), 
                                     x2,-hh,z2, x2/std::sqrt(x2*x2+z2*z2),0,z2/std::sqrt(x2*x2+z2*z2),
                                     x2, hh,z2, x2/std::sqrt(x2*x2+z2*z2),0,z2/std::sqrt(x2*x2+z2*z2) });
        verts.insert(verts.end(), { x1,-hh,z1, x1/std::sqrt(x1*x1+z1*z1),0,z1/std::sqrt(x1*x1+z1*z1),
                                     x2, hh,z2, x2/std::sqrt(x2*x2+z2*z2),0,z2/std::sqrt(x2*x2+z2*z2),
                                     x1, hh,z1, x1/std::sqrt(x1*x1+z1*z1),0,z1/std::sqrt(x1*x1+z1*z1) });
        // Top cap
        verts.insert(verts.end(), { 0,hh,0, 0,1,0, x1,hh,z1, 0,1,0, x2,hh,z2, 0,1,0 });
        // Bottom cap
        verts.insert(verts.end(), { 0,-hh,0, 0,-1,0, x2,-hh,z2, 0,-1,0, x1,-hh,z1, 0,-1,0 });
    }
    
    vert_count = (int)verts.size() / 6;
    
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
}

void Renderer::generate_torus(GLuint& vao, int& vert_count, float major, float minor, int segments, int sides) {
    std::vector<float> verts;
    
    for (int i = 0; i < segments; ++i) {
        float u1 = (float)i / segments * 2.0f * (float)M_PI;
        float u2 = (float)(i+1) / segments * 2.0f * (float)M_PI;
        
        for (int j = 0; j < sides; ++j) {
            float v1 = (float)j / sides * 2.0f * (float)M_PI;
            float v2 = (float)(j+1) / sides * 2.0f * (float)M_PI;
            
            auto point = [&](float u, float v) {
                float x = (major + minor * std::cos(v)) * std::cos(u);
                float y = minor * std::sin(v);
                float z = (major + minor * std::cos(v)) * std::sin(u);
                float nx = std::cos(v) * std::cos(u);
                float ny = std::sin(v);
                float nz = std::cos(v) * std::sin(u);
                verts.insert(verts.end(), { x,y,z, nx,ny,nz });
            };
            
            point(u1, v1); point(u2, v1); point(u2, v2);
            point(u1, v1); point(u2, v2); point(u1, v2);
        }
    }
    
    vert_count = (int)verts.size() / 6;
    
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
}

void Renderer::generate_grid(GLuint& vao, int& vert_count, int size, int divisions) {
    std::vector<float> verts;
    float half = (float)size * 0.5f;
    float step = (float)size / divisions;
    
    for (int i = 0; i <= divisions; ++i) {
        float pos = -half + i * step;
        verts.insert(verts.end(), { pos, 0, -half, 0.3f,0.3f,0.4f });
        verts.insert(verts.end(), { pos, 0,  half, 0.3f,0.3f,0.4f });
        verts.insert(verts.end(), { -half, 0, pos, 0.3f,0.3f,0.4f });
        verts.insert(verts.end(), {  half, 0, pos, 0.3f,0.3f,0.4f });
    }
    
    vert_count = (int)verts.size() / 6;
    
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

// ==================== Shaders ====================
GLuint Renderer::compile_shader(const std::string& vertex_src, const std::string& fragment_src) {
    auto compile = [](GLuint type, const std::string& src) -> GLuint {
        GLuint shader = glCreateShader(type);
        const char* src_ptr = src.c_str();
        glShaderSource(shader, 1, &src_ptr, nullptr);
        glCompileShader(shader);
        return shader;
    };
    
    GLuint vs = compile(GL_VERTEX_SHADER, vertex_src);
    GLuint fs = compile(GL_FRAGMENT_SHADER, fragment_src);
    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    
    return program;
}

std::string Renderer::body_vertex_src() {
    return R"(
    #version 460 core
    layout(location = 0) in vec3 a_pos;
    layout(location = 1) in vec3 a_normal;
    
    uniform mat4 u_model;
    uniform mat4 u_viewProj;
    uniform vec3 u_cameraPos;
    
    out vec3 v_normal;
    out vec3 v_position;
    out vec3 v_viewDir;
    
    void main() {
        vec4 world_pos = u_model * vec4(a_pos, 1.0);
        v_position = world_pos.xyz;
        v_normal = normalize(mat3(u_model) * a_normal);
        v_viewDir = normalize(u_cameraPos - world_pos.xyz);
        gl_Position = u_viewProj * world_pos;
    }
    )";
}

std::string Renderer::body_fragment_src() {
    return R"(
    #version 460 core
    in vec3 v_normal;
    in vec3 v_position;
    in vec3 v_viewDir;
    
    uniform vec3 u_color;
    uniform vec3 u_glowColor;
    uniform float u_emissive;
    uniform float u_opacity;
    uniform vec3 u_lightDir;
    uniform vec3 u_lightColor;
    uniform vec3 u_ambient;
    uniform int u_selected;
    
    out vec4 fragColor;
    
    void main() {
        vec3 N = normalize(v_normal);
        vec3 L = normalize(u_lightDir);
        vec3 V = normalize(v_viewDir);
        
        // Fresnel rim light
        float rim = 1.0 - max(dot(N, V), 0.0);
        rim = pow(rim, 3.0);
        
        // Diffuse
        float diff = max(dot(N, L), 0.0);
        vec3 diffuse = u_color * u_lightColor * diff;
        
        // Ambient
        vec3 ambient = u_color * u_ambient;
        
        // Emissive glow
        vec3 emissive = u_glowColor * u_emissive;
        
        // Rim
        vec3 rimColor = u_glowColor * rim * 0.5;
        
        // Selection outline
        vec3 selection = vec3(0.26, 0.59, 0.98) * u_selected * 0.3;
        
        vec3 finalColor = ambient + diffuse + emissive + rimColor + selection;
        
        // Simple gamma
        finalColor = pow(finalColor, vec3(1.0/2.2));
        
        fragColor = vec4(finalColor, u_opacity);
    }
    )";
}

std::string Renderer::trail_vertex_src() {
    return R"(
    #version 460 core
    layout(location = 0) in vec3 a_pos;
    uniform mat4 u_viewProj;
    void main() {
        gl_Position = u_viewProj * vec4(a_pos, 1.0);
    }
    )";
}

std::string Renderer::trail_fragment_src() {
    return R"(
    #version 460 core
    uniform vec3 u_color;
    uniform float u_opacity;
    out vec4 fragColor;
    void main() {
        fragColor = vec4(u_color, u_opacity);
    }
    )";
}

std::string Renderer::grid_vertex_src() {
    return R"(
    #version 460 core
    layout(location = 0) in vec3 a_pos;
    layout(location = 1) in vec3 a_color;
    uniform mat4 u_viewProj;
    out vec3 v_color;
    void main() {
        v_color = a_color;
        gl_Position = u_viewProj * vec4(a_pos, 1.0);
    }
    )";
}

std::string Renderer::grid_fragment_src() {
    return R"(
    #version 460 core
    in vec3 v_color;
    out vec4 fragColor;
    void main() {
        fragColor = vec4(v_color, 1.0);
    }
    )";
}

std::string Renderer::arrow_vertex_src() {
    return body_vertex_src();
}

std::string Renderer::arrow_fragment_src() {
    return body_fragment_src();
}

} // namespace hlp
