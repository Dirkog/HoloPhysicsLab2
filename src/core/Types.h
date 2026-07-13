#pragma once

// ============================================================
// HoloPhysics Lab 2 — Core Types
// Научная песочница на C++/OpenGL
// ============================================================

#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>

namespace hlp {

// === Double-precision 3D vector ===
using dvec3 = glm::dvec3;
using dmat3 = glm::dmat3;
using dquat = glm::dquat;

// Shape types for bodies
enum class ShapeType : uint8_t {
    Sphere = 0,
    Box,
    Cylinder,
    Torus,
    Capsule,
    Cone,
    Point,
    Custom
};

// Body flags
enum BodyFlags : uint32_t {
    BODY_STATIC    = 1 << 0,
    BODY_CENTRAL   = 1 << 1,
    BODY_GRAVITY   = 1 << 2,  // affected by gravity
    BODY_CHARGED   = 1 << 3,
    BODY_SELECTED  = 1 << 4,
    BODY_VISIBLE   = 1 << 5,
    BODY_HAS_TRAIL = 1 << 6,
};

// Physical properties of a body
struct BodyProperties {
    double mass = 1.0;
    double radius = 0.5;
    double restitution = 0.5;
    double friction = 0.3;
    double charge = 0.0;
    double magnetic_moment = 0.0;
    double temperature = 293.0;  // Kelvin
    double emissivity = 1.0;
};

// Visual properties of a body
struct VisualProperties {
    glm::vec3 color = glm::vec3(0.0f, 1.0f, 0.8f);
    glm::vec3 glow_color = glm::vec3(0.0f, 1.0f, 0.8f);
    float opacity = 0.92f;
    float emissive_intensity = 0.15f;
    ShapeType shape = ShapeType::Sphere;
    std::string label;
    bool wireframe_overlay = false;
};

// === Body state at a point in time ===
struct BodyState {
    dvec3 position{0.0};
    dvec3 velocity{0.0};
    dvec3 angular_velocity{0.0};
    dquat orientation{1.0, 0.0, 0.0, 0.0};
};

// === Physical body ===
class Body {
public:
    uint64_t id;
    BodyState state;
    BodyState prev_state;
    dvec3 force{0.0};
    dvec3 torque{0.0};

    BodyProperties props;
    VisualProperties visual;
    uint32_t flags = BODY_VISIBLE | BODY_GRAVITY;

    std::vector<dvec3> trail;
    size_t max_trail = 500;

    Body(uint64_t id_) : id(id_) {}
    Body() : id(0) {}

    bool is_static() const { return flags & BODY_STATIC; }
    bool is_selected() const { return flags & BODY_SELECTED; }
    bool is_visible() const { return flags & BODY_VISIBLE; }

    double kinetic_energy() const {
        return 0.5 * props.mass * glm::dot(state.velocity, state.velocity);
    }

    double speed() const {
        return glm::length(state.velocity);
    }

    double potential_energy(double g) const {
        return props.mass * std::abs(g) * std::abs(state.position.y);
    }
};

// === Simulation parameters ===
struct SimulationParams {
    // Gravity
    double gravity = 9.81;
    double damping = 0.001;
    double restitution = 0.5;
    double friction = 0.3;
    double time_scale = 1.0;
    
    // Environment
    dvec3 wind{0.0, 0.0, 0.0};
    double air_density = 0.1;
    double buoyancy = 0.0;
    
    // Fields
    double magnetic_field = 0.0;  // Tesla along Y
    double electric_field = 0.0;  // V/m along X
    
    // Forces
    double central_mass = 0.0;    // Central gravitational mass
    double coulomb_k = 0.0;       // Coulomb constant
    double cohesion = 0.0;        // Cohesion strength
    double spring_k = 0.0;        // Global spring constant
    double vortex_strength = 0.0; // Vortex field
    double coriolis = 0.0;        // Coriolis effect
    double turbulence = 0.0;      // Turbulence amplitude
    
    // Enable flags
    bool enable_wind = false;
    bool enable_magnet = false;
    bool enable_coulomb = false;
    bool enable_cohesion = false;
    bool enable_vortex = false;
    bool enable_coriolis = false;
    bool enable_central_g = false;
    bool enable_turbulence = false;
    bool enable_buoyancy = false;
    bool enable_springs = false;
    bool enable_drag = true;
    bool show_trails = true;
    bool show_grid = true;
    bool show_axes = true;
    bool show_forces = false;
    bool show_velocity_vec = false;
    bool pause_on_collision = false;
};

// === Integration method ===
enum class IntegratorType : uint8_t {
    Euler = 0,
    SymplecticEuler,    // Semi-implicit Euler
    Verlet,             // Velocity Verlet
    RK4,                // Classical Runge-Kutta 4
    RK5,                // Runge-Kutta-Fehlberg
    Ruth,               // 3rd order symplectic
};

// === Collision event ===
struct CollisionEvent {
    uint64_t body_a;
    uint64_t body_b;
    dvec3 point;
    dvec3 normal;
    double impulse_magnitude;
    double timestamp;
};

// === Spring constraint ===
struct SpringConstraint {
    uint64_t body_a;
    uint64_t body_b;
    double rest_length;
    double stiffness;
    double damping;
};

// === Force callback ===
struct ForceContext {
    const SimulationParams& params;
    std::vector<Body>& bodies;
    double dt;
    double time;
};

// JSON serialization
void to_json(nlohmann::json& j, const dvec3& v);
void from_json(const nlohmann::json& j, dvec3& v);
void to_json(nlohmann::json& j, const Body& b);
void from_json(const nlohmann::json& j, Body& b);
void to_json(nlohmann::json& j, const SimulationParams& p);
void from_json(const nlohmann::json& j, SimulationParams& p);

} // namespace hlp
