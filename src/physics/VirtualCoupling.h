#pragma once

// ============================================================
// HoloPhysics Lab 2 — Virtual Coupling System
// 
// Решает проблему "God Hand" (бесконечная масса руки):
// Вместо прямого маппинга руки в физику, создаётся
// PD-контроллер между "Target" (CV) и "Physical" (RigidBody).
// 
// Это гарантирует сохранение энергии и предотвращает
// "взрывы" физики при быстрых движениях руки.
// ============================================================

#include "../core/Types.h"
#include "../core/Engine.h"
#include "../physics/ConstraintSolver.h"
#include "../input/VirtualHand.h"
#include <array>

namespace hlp {

// PD Controller gains
struct VirtualCouplingConfig {
    // Proportional gain (spring stiffness)
    double kp_position = 200.0;    // N/m - how tightly hand follows target
    double kp_rotation = 10.0;     // N·m/rad
    
    // Derivative gain (damping)
    double kd_velocity = 20.0;     // N·s/m - how much to dampen
    double kd_rotation = 5.0;      // N·m·s/rad
    
    // Mass of virtual hand
    double hand_mass = 0.5;         // kg - finite mass prevents infinite impulse
    double hand_radius = 0.08;      // m - collision radius
    
    // Limits
    double max_force = 50.0;        // N - máximo force the hand can exert
    double max_correction_speed = 20.0; // m/s - clamp to prevent explosions
    
    // Virtual hand body ID
    uint64_t virtual_body_id = UINT64_MAX - 1;
};

struct VirtualHandBody {
    BodyState state;        // Physical state (with mass, velocity)
    BodyState target;       // Target state from CV
    dvec3 force{0};
    
    // PD error tracking
    dvec3 prev_error{0};
    dvec3 error_derivative{0};
    
    // Mass and collision
    double mass = 0.5;
    double radius = 0.08;
    bool enabled = false;
};

class VirtualCoupling {
public:
    VirtualCoupling(Engine& engine);
    ~VirtualCoupling();

    // === Main update ===
    void update(const VirtualHand& hand, double dt);
    
    // === Access ===
    const VirtualHandBody& hand_body(int idx) const { return hand_bodies_[idx]; }
    VirtualHandBody& hand_body(int idx) { return hand_bodies_[idx]; }
    bool is_active() const { return active_; }
    
    // === Force feedback to host (for haptics) ===
    dvec3 applied_force() const { return last_force_; }
    double penetration_depth() const { return max_penetration_; }

    // === Configuration ===
    void set_config(const VirtualCouplingConfig& cfg) { config_ = cfg; }
    const VirtualCouplingConfig& config() const { return config_; }
    VirtualCouplingConfig& config() { return config_; }
    
    // === Collision query ===
    // Returns bodies currently touching the virtual hand
    const std::vector<uint64_t>& contacting_bodies() const { return contacting_; }

private:
    Engine& engine_;
    VirtualCouplingConfig config_;
    bool active_ = false;
    
    // Two hands (right, left)
    std::array<VirtualHandBody, 2> hand_bodies_;
    
    // PD controller for single hand
    void compute_pd_force(VirtualHandBody& hb, const dvec3& target_pos,
                          const dvec3& target_vel, double dt);
    
    // Collision detection for virtual hand
    void detect_hand_collisions(VirtualHandBody& hb);
    
    // Apply forces to contacting bodies
    void apply_contact_forces(VirtualHandBody& hb, double dt);
    
    // State
    dvec3 last_force_{0};
    double max_penetration_ = 0;
    std::vector<uint64_t> contacting_;
    
    // Body state management
    void ensure_body_exists(int hand_idx);
    void sync_body_with_scene(int hand_idx);
};

// ==================== Soft Grab Controller ====================

struct SoftGrabConfig {
    double grab_radius = 0.5;       // Max distance to initiate grab
    double release_radius = 0.7;    // Distance to release (hysteresis)
    double grab_stiffness = 50.0;   // Stiffness of grab constraint
    double release_velocity_gain = 1.2; // Throw velocity multiplier
    
    // Virtual coupling integration
    bool use_coupling = true;       // Use PD controller for smooth grab
    double coupling_blend = 0.8;    // Blend between direct and PD control
};

class SoftGrabController {
public:
    SoftGrabController() = default;
    
    // State
    enum GrabState {
        IDLE,           // Nothing happening
        REACHING,       // Hand approaching object
        GRABBING,       // Pinch active, dragging
        HOLDING,        // Object held, hand moving
        RELEASING,      // Pinch released, throw
    };
    
    struct GrabData {
        GrabState state = IDLE;
        uint64_t grabbed_body = UINT64_MAX;
        dvec3 grab_offset{0};       // Offset between hand and body COM
        dvec3 release_velocity{0};  // Velocity on release
        double grab_time = 0;
        double hold_distance = 0;
    };
    
    // Update grab state
    GrabData update(const VirtualHand& hand, 
                    const VirtualHandBody& hand_body,
                    const std::vector<Body>& scene_bodies,
                    double dt);
    
    // Apply spring force to grabbed body
    void apply_grab_force(Body& body, const GrabData& grab,
                          const dvec3& hand_pos, const dvec3& hand_vel,
                          double dt);
    
    // Configuration
    void set_config(const SoftGrabConfig& cfg) { config_ = cfg; }
    const SoftGrabConfig& config() const { return config_; }
    
private:
    SoftGrabConfig config_;
    GrabData grab_data_;
    
    // Find nearest grabbable body
    uint64_t find_nearest(const dvec3& point, double radius,
                          const std::vector<Body>& bodies);
    
    // Spring force calculation
    dvec3 compute_spring_force(const Body& body, const dvec3& hand_pos,
                               const dvec3& hand_vel, 
                               const dvec3& offset, double dt);
};

} // namespace hlp
