#pragma once

// ============================================================
// HoloPhysics Lab 2 — Main Engine
// Связывает физику, рендеринг и UI
// ============================================================

#include "Scene.h"
#include "Solver.h"
#include <vector>

namespace hlp {

struct EngineStats {
    double fps = 60.0;
    double sim_time = 0.0;
    size_t body_count = 0;
    size_t spring_count = 0;
    double total_energy = 0.0;
    double momentum = 0.0;
    double energy_drift = 0.0;
    IntegratorType integrator = IntegratorType::SymplecticEuler;
    int step_count = 0;
    double substeps = 1;
};

class Engine {
public:
    Engine();
    ~Engine();

    // Scene management
    Scene& scene() { return *scene_; }
    const Scene& scene() const { return *scene_; }

    // Physics step
    void step_physics();
    void set_substeps(int n) { substeps_ = n; }
    int substeps() const { return substeps_; }

    // Forces
    void apply_force(uint64_t body_id, const dvec3& force);
    void apply_impulse(uint64_t body_id, const dvec3& impulse);

    // Explosion
    void explode(const dvec3& center, double radius, double force);

    // Gravity well (toggle)
    void set_gravity_well(bool active) { gravity_well_active_ = active; }
    bool gravity_well_active() const { return gravity_well_active_; }
    void set_gravity_well_point(const dvec3& pt) { gravity_well_point_ = pt; }
    void set_gravity_well_strength(double s) { gravity_well_strength_ = s; }

    // Pause
    void set_paused(bool p) { paused_ = p; }
    bool paused() const { return paused_; }

    // Stats
    const EngineStats& stats() const { return stats_; }
    void reset_stats();

    // Collision events
    const std::vector<CollisionEvent>& collision_events() const { return collision_events_; }
    void clear_collision_events() { collision_events_.clear(); }

    // Random body spawn
    void spawn_random_bodies(int count);

    // Freeze / randomize
    void freeze_all();
    void randomize_velocities(double magnitude = 3.0);

private:
    std::unique_ptr<Scene> scene_;
    PhysicsSystem physics_;
    Solver solver_;
    
    EngineStats stats_;
    int substeps_ = 2;
    bool paused_ = false;
    bool gravity_well_active_ = false;
    dvec3 gravity_well_point_{0.0, 5.0, 0.0};
    double gravity_well_strength_ = 15.0;
    
    std::vector<CollisionEvent> collision_events_;
    double event_clear_time_ = 0.0;
};

} // namespace hlp
