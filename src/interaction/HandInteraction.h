#pragma once

// ============================================================
// HoloPhysics Lab 2 — Hand Interaction System
// Мост между жестами рук и физическим движком
// ============================================================

#include "../core/Engine.h"
#include "../core/Scene.h"
#include "../physics/ConstraintSolver.h"
#include "../input/VirtualHand.h"
#include <memory>

namespace hlp {

struct InteractionConfig {
    double grab_radius = 0.5;          // Max distance for grab (meters)
    double grab_stiffness = 200.0;     // Stiffness of grab constraint
    double grab_damping = 10.0;
    double throw_gain = 1.2;           // Velocity multiplier on throw
    double repulsion_radius = 0.8;     // Fist repulsion radius
    double repulsion_force = 15.0;     // Fist repulsion strength
    double raycast_max_dist = 15.0;    // Pointing raycast distance
    double force_field_strength = 5.0; // Palm force field
    bool enable_debug_visuals = true;
};

enum class InteractionMode {
    GRAB,            // Pinch → constraint-based grab
    FORCE_FIELD,     // Palm creates repulsion/attraction field
    RAY_SELECT,      // Pointing raycast for telekinesis
    MIXED            // All of the above
};

class HandInteractionSystem {
public:
    HandInteractionSystem(Engine& engine, ConstraintSolver& constraint_solver);
    ~HandInteractionSystem();

    // === Main update ===
    void update(const GestureController& gesture_ctrl, double dt);

    // === Configuration ===
    void set_config(const InteractionConfig& cfg) { config_ = cfg; }
    const InteractionConfig& config() const { return config_; }
    InteractionConfig& config() { return config_; }
    
    void set_mode(InteractionMode mode) { mode_ = mode; }
    InteractionMode mode() const { return mode_; }

    // === State queries ===
    uint64_t highlighted_body() const { return highlighted_body_; }
    uint64_t grabbed_body() const { return grabbed_body_[0]; }
    bool is_grabbing() const { return is_grabbing_[0]; }
    int active_constraints() const { return grab_constraint_count_; }
    
    // === Debug visualization ===
    struct DebugInfo {
        dvec3 interaction_point;
        dvec3 ray_end;
        float grab_highlight_radius;
        bool show_ray;
        bool show_force_field;
        std::vector<uint64_t> affected_bodies;
    };
    const DebugInfo& debug() const { return debug_; }

    // === Force application helpers (for scripting) ===
    void apply_force_at_point(const dvec3& point, const dvec3& force, double radius);
    void grab_body_at_point(uint64_t body_id);
    void release_all();

private:
    Engine& engine_;
    ConstraintSolver& constraints_;
    InteractionConfig config_;
    InteractionMode mode_ = InteractionMode::MIXED;
    DebugInfo debug_;

    // Per-hand state
    static constexpr int MAX_HANDS = 2;
    bool is_grabbing_[MAX_HANDS] = {false, false};
    uint64_t grabbed_body_[MAX_HANDS] = {UINT64_MAX, UINT64_MAX};
    int grab_constraint_idx_[MAX_HANDS] = {-1, -1};
    dvec3 last_hand_pos_[MAX_HANDS];
    int grab_constraint_count_ = 0;

    // Internal methods
    void process_hand(VirtualHand& hand, int hand_idx, double dt);
    
    void try_grab(VirtualHand& hand, int hand_idx);
    void update_grab(VirtualHand& hand, int hand_idx, double dt);
    void release_grab(VirtualHand& hand, int hand_idx);
    void apply_throw(VirtualHand& hand, int hand_idx);
    
    void apply_repulsion(VirtualHand& hand);
    void apply_force_field(VirtualHand& hand);
    void raycast_select(VirtualHand& hand);
    
    uint64_t find_nearest_body(const dvec3& point, double radius, 
                                uint64_t exclude = UINT64_MAX);
};

} // namespace hlp
