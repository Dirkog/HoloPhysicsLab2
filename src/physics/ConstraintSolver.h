#pragma once

// ============================================================
// HoloPhysics Lab 2 — Constraint Solver (PBD/XPBD)
// Extended Position Based Dynamics for joints and connections
// ============================================================

#include "../core/Types.h"
#include <vector>
#include <functional>

namespace hlp {

// ==================== Constraint Types ====================

enum class ConstraintType : uint8_t {
    DISTANCE,        // Fixed distance between two bodies
    BALL_SOCKET,     // Spherical joint (point constraint)
    HINGE,           // Hinge joint (rotation around one axis)
    SLIDER,          // Slider/prismatic joint (movement along one axis)
    FIXED,           // Rigid connection (no relative motion)
    SPRING,          // Spring-damper
    ANGULAR_SPRING,  // Angular spring (torsion)
    CUSTOM           // User-defined constraint
};

// Distance constraint params
struct DistanceParams {
    double rest_length;
    double stiffness = 0.8;
};

// Hinge joint params
struct HingeParams {
    dvec3 axis_a{1, 0, 0};    // Axis in body A's local frame
    dvec3 axis_b{1, 0, 0};    // Axis in body B's local frame
    dvec3 anchor_a{0, 0, 0};  // Anchor point in body A's local frame
    dvec3 anchor_b{0, 0, 0};  // Anchor point in body B's local frame
    double angle_min = -3.14159;
    double angle_max = 3.14159;
    double stiffness = 0.9;
};

// Slider joint params
struct SliderParams {
    dvec3 axis{1, 0, 0};      // Sliding axis
    double limit_min = -10.0;
    double limit_max = 10.0;
    double stiffness = 0.8;
};

// Fixed joint params
struct FixedParams {
    dvec3 anchor_a{0, 0, 0};
    dquat orientation_offset{1, 0, 0, 0};
    double stiffness = 0.95;
};

// Spring parameters
struct SpringParams2 {
    double rest_length;
    double stiffness = 50.0;
    double damping = 1.0;
};

// Angular spring params
struct AngularSpringParams {
    dquat rest_orientation{1, 0, 0, 0};
    double stiffness = 10.0;
    double damping = 0.5;
};

// Union of all constraint parameters
struct ConstraintParams {
    ConstraintType type;
    uint64_t body_a, body_b;
    double compliance = 0.0;  // 0 = rigid, >0 = soft
    
    union {
        DistanceParams distance;
        HingeParams hinge;
        SliderParams slider;
        FixedParams fixed;
        SpringParams2 spring;
        AngularSpringParams angular_spring;
    };
    
    // Active flag
    bool active = true;
    bool breakable = false;
    double break_force = 1e10;
};

// ==================== Constraint Solver ====================

class ConstraintSolver {
public:
    ConstraintSolver();
    ~ConstraintSolver();

    // === Constraint management ===
    int add_constraint(const ConstraintParams& params);
    void remove_constraint(int index);
    void clear_constraints();
    ConstraintParams* get_constraint(int index);
    const std::vector<ConstraintParams>& constraints() const { return constraints_; }
    
    // === Main solve ===
    // PBD iteration: predict positions, solve constraints, update velocities
    void solve(std::vector<Body>& bodies, double dt, int iterations = 5);
    
    // Single constraint solve (for warm starting)
    void solve_constraint(std::vector<Body>& bodies, int index, double dt);
    
    // === Accessors ===
    int constraint_count() const { return (int)constraints_.size(); }
    int active_constraint_count() const;
    
    // === Gradient computation (for stiffness -> compliance conversion) ===
    static double stiffness_to_compliance(double stiffness, double dt);
    
private:
    std::vector<ConstraintParams> constraints_;
    
    // PBD solve helpers
    void solve_distance(std::vector<Body>& bodies, const ConstraintParams& c);
    void solve_ball_socket(std::vector<Body>& bodies, const ConstraintParams& c);
    void solve_hinge(std::vector<Body>& bodies, const ConstraintParams& c);
    void solve_slider(std::vector<Body>& bodies, const ConstraintParams& c);
    void solve_fixed(std::vector<Body>& bodies, const ConstraintParams& c);
    void solve_spring(std::vector<Body>& bodies, const ConstraintParams& c, double dt);
    void solve_angular_spring(std::vector<Body>& bodies, const ConstraintParams& c, double dt);
    
    // Utility
    dvec3 get_world_anchor(const Body& body, const dvec3& local_anchor);
    double compute_constraint_error(const ConstraintParams& c, 
                                    const std::vector<Body>& bodies);
};

// ==================== Soft Body / Cloth Constraints ====================

class SoftBodySystem {
public:
    SoftBodySystem() = default;
    
    // Create a grid of connected bodies with spring constraints
    void create_cloth(std::vector<Body>& bodies,
                      const dvec3& origin, int width, int depth,
                      double spacing, double mass, 
                      bool pin_corners = true);
    
    // Create a chain/rope of connected bodies
    void create_chain(std::vector<Body>& bodies,
                      const dvec3& start, const dvec3& end,
                      int segments, double mass, double radius);
    
    // Create a soft body (jelly) from a central body surrounded by shell
    void create_jelly(std::vector<Body>& bodies,
                      const dvec3& center, double radius,
                      int shell_particles, double mass);
    
    // Get the generated constraints
    std::vector<ConstraintParams>& constraints() { return constraints_; }
    
private:
    std::vector<ConstraintParams> constraints_;
    
    void add_bend_constraint(std::vector<Body>& bodies,
                            int i1, int i2, int i3);
};

} // namespace hlp
