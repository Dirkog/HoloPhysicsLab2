#pragma once

// ============================================================
// HoloPhysics Lab 2 — OpenGL Renderer
// Научная визуализация с минималистичным дизайном
// ============================================================

#include "../core/Types.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <string>
#include <cstdint>

// Forward declare GL types
typedef unsigned int GLuint;
typedef int GLint;

namespace hlp {

struct Light {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    bool init(int width, int height);
    void resize(int width, int height);
    void shutdown();

    // Camera
    void set_camera(const glm::vec3& eye, const glm::vec3& target);
    void orbit_camera(float delta_x, float delta_y);
    void zoom_camera(float delta);
    void pan_camera(float delta_x, float delta_y);
    
    glm::mat4 view_matrix() const;
    glm::mat4 proj_matrix() const;
    glm::vec3 camera_pos() const { return camera_eye_; }

    // Rendering
    void begin_frame();
    void end_frame();
    
    void render_scene(const std::vector<Body>& bodies, 
                      const SimulationParams& params,
                      const std::vector<CollisionEvent>& events);
    
    void render_grid(const SimulationParams& params);
    void render_trails(const std::vector<Body>& bodies);
    void render_forces(const std::vector<Body>& bodies);
    void render_collision_events(const std::vector<CollisionEvent>& events, double time);
    void render_selection_highlight(const Body* selected);

    // Selection (ray casting)
    int pick_body(int screen_x, int screen_y, int viewport_w, int viewport_h,
                  const std::vector<Body>& bodies);

    // Clear color
    void set_clear_color(const glm::vec3& color) { clear_color_ = color; }

private:
    // OpenGL resources
    GLuint shader_body_ = 0;
    GLuint shader_trail_ = 0;
    GLuint shader_grid_ = 0;
    GLuint shader_arrow_ = 0;
    GLuint shader_ring_ = 0;
    
    GLuint vao_sphere_ = 0;
    GLuint vao_box_ = 0;
    GLuint vao_cylinder_ = 0;
    GLuint vao_torus_ = 0;
    GLuint vao_grid_ = 0;
    GLuint vao_quad_ = 0;
    
    int sphere_verts_ = 0;
    int box_verts_ = 0;
    int cylinder_verts_ = 0;
    int torus_verts_ = 0;
    int grid_verts_ = 0;

    // Window
    int width_ = 1600;
    int height_ = 1000;
    glm::vec3 clear_color_ = glm::vec3(0.04f, 0.04f, 0.06f);
    
    // Camera
    glm::vec3 camera_eye_ = glm::vec3(12.0f, 8.0f, 12.0f);
    glm::vec3 camera_target_ = glm::vec3(0.0f, 2.0f, 0.0f);
    glm::vec3 camera_up_ = glm::vec3(0.0f, 1.0f, 0.0f);
    float camera_distance_ = 18.0f;
    float camera_theta_ = 0.785f;  // 45 degrees
    float camera_phi_ = 0.615f;     // ~35 degrees
    float fov_ = 50.0f;
    float near_plane_ = 0.1f;
    float far_plane_ = 500.0f;

    // Helper methods
    GLuint compile_shader(const std::string& vertex_src, const std::string& fragment_src);
    void generate_sphere(GLuint& vao, int& vert_count, float radius = 1.0f, int slices = 24, int stacks = 18);
    void generate_box(GLuint& vao, int& vert_count, float size = 1.0f);
    void generate_cylinder(GLuint& vao, int& vert_count, float radius = 1.0f, float height = 1.0f, int slices = 24);
    void generate_torus(GLuint& vao, int& vert_count, float major = 1.0f, float minor = 0.4f, int segments = 24, int sides = 16);
    void generate_grid(GLuint& vao, int& vert_count, int size = 20, int divisions = 20);
    void generate_arrow_geometry(GLuint& vao, int& vert_count);
    
    // Shader sources
    std::string body_vertex_src();
    std::string body_fragment_src();
    std::string trail_vertex_src();
    std::string trail_fragment_src();
    std::string grid_vertex_src();
    std::string grid_fragment_src();
    std::string arrow_vertex_src();
    std::string arrow_fragment_src();
};

} // namespace hlp
